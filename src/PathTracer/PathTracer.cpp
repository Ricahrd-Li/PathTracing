#include "PathTracer.h"

#include <UBL/Image.h>

#include <iostream>

#include <thread>
#include <chrono>
#include <ctime>   
#include <cmath>

#define PI 3.14159

using namespace Ubpa;
using namespace std;

PathTracer::PathTracer(const Scene* scene, const SObj* cam_obj, Image* img)
	: scene{ scene },
	bvh{ const_cast<Scene*>(scene) },
	img{ img },
	cam{ cam_obj->Get<Cmpt::Camera>() },
	ccs{ cam->GenCoordinateSystem(cam_obj->Get<Cmpt::L2W>()->value) }
{
	IntersectorVisibility::Instance();
	IntersectorClosest::Instance();

	scene->Each([this](const Cmpt::Light* light) ->bool {
		if (!vtable_is<EnvLight>(light->light.get()))
			return true; // continue

		env_light = static_cast<const EnvLight*>(light->light.get());
		return false; // stop
	});

	// TODO: preprocess env_light here
	// create alias chart here!
	cout << "Path tracing initializing." << endl;
	float env_light_illumination_sum = 0.0;

	std::vector<std::tuple<float, std::pair<int, int>>> queue_p_greater_than_one;
	std::vector<std::tuple<float, std::pair<int, int>>> queue_p_leq_one;

	auto env_h = env_light->texture->img->height;
	auto env_w = env_light->texture->img->width;

	env_light_alias_table_p.resize(env_h* env_w);
	env_light_alias_table_idx.resize(env_h* env_w);

	// first loop: to get illumination sum
	for (int i = 0; i < env_h; i++)
	{
		for (int j = 0; j < env_w; j++)
		{
			env_light_illumination_sum += env_light->texture->img->At(j, i).to_rgb().illumination();
		}
	}

	// second loop: to get queue_p_greater_than_one & queue_p_leq_one
	for (int i = 0; i < env_h; i++)
	{
		for (int j = 0; j < env_w; j++)
		{
			float p = float(env_light->texture->img->At(j, i).to_rgb().illumination() * env_h * env_w) / env_light_illumination_sum;
			env_light_pdf[{i, j}] = p; // store probability 

			//p = p * env_h * env_w;
			if (p > 1) queue_p_greater_than_one.push_back({ p, {i,j} });
			else queue_p_leq_one.push_back({ p, {i,j} });
		}
	}

	// Third loop: to build alias table (env_light_alias_table_p & env_light_alias_table_idx)
	while (!queue_p_greater_than_one.empty() && !queue_p_leq_one.empty())
	{
		auto p_leq_one_info = queue_p_leq_one.back();
		queue_p_leq_one.pop_back();

		auto idx = std::get<1>(p_leq_one_info);
		auto p = std::get<0>(p_leq_one_info);
		if (fabs(p - 1.0) < 1e-5)
		{ // p equals to 1 
			env_light_alias_table_p[std::get<0>(idx) * env_w + std::get<1>(idx)] = { 1.0 , 0.0 };
			env_light_alias_table_idx[std::get<0>(idx) * env_w + std::get<1>(idx)] = { idx, {-1, -1} }; // -1 means null 
		}
		else
		{ // p less than 1 
			auto p_gt_one_info = queue_p_greater_than_one.back();
			auto p2 = std::get<0>(p_gt_one_info);
			queue_p_greater_than_one.pop_back();

			auto idx2 = std::get<1>(p_gt_one_info);

			env_light_alias_table_p[std::get<0>(idx) * env_w + std::get<1>(idx)] = { p , 1 - p };
			env_light_alias_table_idx[std::get<0>(idx) * env_w + std::get<1>(idx)] = { idx, idx2 };

			if (p2 - (1 - p) > 1) queue_p_greater_than_one.push_back({ p2 - (1 - p), idx2 });
			else queue_p_leq_one.push_back({ p2 - (1 - p), idx2 });
		}
	}
	while (!queue_p_leq_one.empty())
	{
		auto p_leq_one_info = queue_p_leq_one.back();
		queue_p_leq_one.pop_back();
		auto idx = std::get<1>(p_leq_one_info);
		env_light_alias_table_p[std::get<0>(idx) * env_w + std::get<1>(idx)] = { 1.0 , 0.0 };
		env_light_alias_table_idx[std::get<0>(idx) * env_w + std::get<1>(idx)] = { idx,{-1, -1} }; // -1 means null 
	}
	while (!queue_p_greater_than_one.empty())
	{
		auto p_gt_one_info = queue_p_greater_than_one.back();
		queue_p_greater_than_one.pop_back();
		auto idx = std::get<1>(p_gt_one_info);
		env_light_alias_table_p[std::get<0>(idx) * env_w + std::get<1>(idx)] = { 1.0 , 0.0 };
		env_light_alias_table_idx[std::get<0>(idx) * env_w + std::get<1>(idx)] = { idx, {-1, -1} }; // -1 means null 
	}
	assert(queue_p_leq_one.empty());
	assert(queue_p_greater_than_one.empty());
}

void PathTracer::Run() {
	cout << "Path tracing starts. " << endl;
	img->SetAll(0.f);

	const size_t spp = 40; // samples per pixel
	auto start_t = std::chrono::system_clock::now();
#ifdef NDEBUG


	const size_t core_num = std::thread::hardware_concurrency();
	auto work = [this, core_num, spp](size_t id) {
		for (size_t j = id; j < img->height; j += core_num) {
			for (size_t i = 0; i < img->width; i++) {
				for (size_t k = 0; k < spp; k++) {
					float u = (i + rand01<float>() - 0.5f) / img->width;
					float v = (j + rand01<float>() - 0.5f) / img->height;
					rayf3 r = cam->GenRay(u, v, ccs);
					rgbf Lo;
					do { Lo = Shade(IntersectorClosest::Instance().Visit(&bvh, r), -r.dir, true); } while (Lo.has_nan());
					img->At<rgbf>(i, j) += Lo / float(spp);
				}
			}
			float progress = (j + 1) / float(img->height);
			cout << progress << endl;
		}
	};
	vector<thread> workers;
	for (size_t i = 0; i < core_num; i++)
		workers.emplace_back(work, i);
	for (auto& worker : workers)
		worker.join();

#else
	//Intersectors intersectors;
	for (size_t j = 0; j < img->height; j++) {
		for (size_t i = 0; i < img->width; i++) {
			for (size_t k = 0; k < spp; k++) {
				float u = (i + rand01<float>() - 0.5f) / img->width;
				float v = (j + rand01<float>() - 0.5f) / img->height;
				rayf3 r = cam->GenRay(u, v, ccs);
				rgbf Lo;
				do { Lo = Shade(IntersectorClosest::Instance().Visit(&bvh, r), -r.dir, true); } while (Lo.has_nan());
				img->At<rgbf>(i, j) += Lo / spp;
			}
		}
		float progress = (j + 1) / float(img->height);
		cout << progress << endl;
	}
#endif
	auto end_t = std::chrono::system_clock::now();
	std::chrono::duration<double> elapsed_seconds = end_t - start_t;
	std::time_t end_time = std::chrono::system_clock::to_time_t(end_t);

	std::cout << "finished computation at " << std::ctime(&end_time)
		<< "use time: " << elapsed_seconds.count() << "s\n";
}

template <typename T> T pair_max(const T a, const T b)
{
	return a > b ? a : b;
}

rgbf PathTracer::Shade(const IntersectorClosest::Rst& intersection, const vecf3& wo, bool last_bounce_specular) const {
	// TODO: HW9 - Trace
	// [ Tips ]
	// - EnvLight::Radiance(<direction>), <direction> is pointing to environment light
	// - AreaLight::Radiance(<uv>)
	// - rayf3: point, dir, tmin, **tmax**
	// - IntersectorVisibility::Instance().Visit(&bvh, <rayf3>)
	//   - tmin = EPSILON<float>
	//   - tmax = distance to light - EPSILON<float>
	// - IntersectorClosest::Instance().Visit(&bvh, <rayf3>)
	//   - tmin as default (EPSILON<float>)
	//   - tmax as default (FLT_MAX)
	//
	// struct IntersectorClosest::Rst {
	//	 bool IsIntersected() const noexcept { return sobj != nullptr; }
	//	 const SObj* sobj{ nullptr }; // intersection sobj
	//	 pointf3 pos; // intersection point's position
	//	 pointf2 uv; // texcoord
	//   normalf n; // normal, normalized
	//	 vecf3 tangent; // perpendicular to normal, normalized
	// };

	constexpr rgbf error_color = rgbf{ 1.f,0.f,1.f };
	constexpr rgbf todo_color = rgbf{ 0.f,1.f,0.f };
	constexpr rgbf zero_color = rgbf{ 0.f,0.f,0.f };

	// 1. if not intersecting with light or objects: then must be env light
	if (!intersection.IsIntersected()) {
		if (last_bounce_specular && env_light != nullptr) {
			return env_light->Radiance(-wo.normalize()); // need to add minus to get the direction of wi to make background right.
		}
		else
			return zero_color;
	}

	// 2. if not intersecting with objects, then should intersecting with light  
	if (!intersection.sobj->Get<Cmpt::Material>()) {
		auto light = intersection.sobj->Get<Cmpt::Light>();
		if (!light) return error_color;

		if (last_bounce_specular) { // avoid double-count
			auto area_light = dynamic_cast<const AreaLight*>(light->light.get());
			if (!area_light) return error_color;

			// TODO: area light
			return area_light->Radiance(intersection.uv);

		}
		else
			return zero_color;
	}

	// 3. if intersecting with objects, need to get direct light and indirect light
	rgbf L_dir{ 0.f };
	rgbf L_indir{ 0.f };

	scene->Each([=, &L_dir](const Cmpt::Light* light, const Cmpt::L2W* l2w, const Cmpt::SObjPtr* ptr) {
		// TODO: L_dir += ...
		// - use PathTracer::BRDF to get BRDF value
		//SampleLightResult sample_light_rst = SampleLight(intersection, wo, light, l2w, ptr, env_light_pdf, env_light_alias_table_p, env_light_alias_table_idx);
		SampleLightResult sample_light_rst = SampleLight(intersection, wo, light, l2w, ptr);
		if (sample_light_rst.pd <= 0)
			return;
		if (sample_light_rst.is_infinity) {  // infinity means the light is from environment 
			// TODO: L_dir of environment light

			// check visibility 
			auto new_intersection = IntersectorClosest::Instance().Visit(&bvh, rayf3(intersection.pos, (-sample_light_rst.n).cast_to<vecf3>()));
			if (!new_intersection.IsIntersected())
			{
				auto fr = BRDF(intersection, (-sample_light_rst.n).cast_to<vecf3>(), wo.normalize());
				L_dir += sample_light_rst.L * fr * pair_max<float>(intersection.n.dot(-sample_light_rst.n), 0.0) / sample_light_rst.pd;
			}
			else return;
		}
		else {
			// TODO: L_dir of area light
			auto wi = (sample_light_rst.x - intersection.pos);
			// check visibility 
			if (!IntersectorVisibility::Instance().Visit(&bvh, rayf3(sample_light_rst.x, -wi.normalize(), EPSILON<float>, wi.norm() - EPSILON<float>)))
			{
				return;
			}
			else
			{
				auto fr = BRDF(intersection, wi.cast_to<vecf3>().normalize(), wo.normalize());

				L_dir += sample_light_rst.L * fr * pair_max<float>(sample_light_rst.n.dot(-wi.normalize().cast_to<normalf>()), 0.0)
					* pair_max<float>(intersection.n.dot(wi.normalize().cast_to<normalf>()), 0.0) / wi.norm2() / sample_light_rst.pd;
			}
		}
	});

	// TODO: Russian Roulette testing 
	float ksi = rand01<float>();
	float p_rr = 0.7;
	if (ksi > p_rr) return L_dir;

	// TODO: recursion
	// - use PathTracer::SampleBRDF to get wi and pd (probability density)
	// wi may be **under** the surface
	// - use PathTracer::BRDF to get BRDF value

	auto sample_brdf = SampleBRDF(intersection, wo);
	auto brdf_wi = std::get<0>(sample_brdf).normalize();
	auto brdf_pd = std::get<1>(sample_brdf);

	auto new_ray = rayf3(intersection.pos, brdf_wi);

	auto new_intersection = IntersectorClosest::Instance().Visit(&bvh, new_ray);
	auto fr = BRDF(intersection, brdf_wi, wo.normalize()); // get fr
	auto cos = pair_max<float>(intersection.n.dot(brdf_wi.cast_to<normalf>()), 0.0); // get cos 

	if (!new_intersection.IsIntersected())
	{
		L_indir = env_light->Radiance(brdf_wi) * fr * cos / brdf_pd / p_rr; // env_light
	}
	else if (new_intersection.sobj->Get<Cmpt::Material>()) // intersect with non-emitting object, else if intersect with light, then should be direct light 
	{
		L_indir = Shade(new_intersection, -brdf_wi, last_bounce_specular) * fr * cos / brdf_pd / p_rr;
	}
	// TODO: combine L_dir and L_indir
	return L_dir + L_indir;
}

//PathTracer::SampleLightResult PathTracer::SampleLight(IntersectorClosest::Rst intersection, const vecf3& wo, const Cmpt::Light* light, const Cmpt::L2W* l2w, const Cmpt::SObjPtr* ptr, 
//																				std::map<std::pair<int, int>, float> env_light_pdf, std::vector<std::pair<float, float>> env_light_alias_table_p,
PathTracer::SampleLightResult PathTracer::SampleLight(IntersectorClosest::Rst intersection, const vecf3& wo, const Cmpt::Light* light, const Cmpt::L2W* l2w, const Cmpt::SObjPtr* ptr) const
{
	PathTracer::SampleLightResult rst;
	if (vtable_is<AreaLight>(light->light.get())) {
		auto area_light = static_cast<const AreaLight*>(light->light.get());
		auto geo = ptr->value->Get<Cmpt::Geometry>();
		if (!geo) return rst; // invalid
		if (!vtable_is<Square>(geo->primitive.get())) return rst; // not support

		auto Xi = uniform_in_square<float>(); // [0, 1] x [0, 1]
		pointf3 pos_light_space{ 2 * Xi[0] - 1, 0, 2 * Xi[1] - 1 };
		scalef3 s = l2w->WorldScale();
		float area = s[0] * s[1] * Square::area;

		rst.L = area_light->Radiance(Xi.cast_to<pointf2>());
		rst.pd = 1.f / area;
		rst.x = l2w->value * pos_light_space;
		rst.is_infinity = false;
		rst.n = l2w->UpInWorld().cast_to<normalf>();
	}
	else if (vtable_is<EnvLight>(light->light.get())) {
		rst.is_infinity = true;

		auto mat = intersection.sobj->Get<Cmpt::Material>();
		if (!mat) return rst; // invalid
		auto brdf = dynamic_cast<const stdBRDF*>(mat->material.get());
		if (!brdf) return rst; // not support

		// multi-importance sampling, MIS

		auto env_light = static_cast<const EnvLight*>(light->light.get());

		float metalness = brdf->Metalness(intersection.uv);
		float roughness = brdf->Roughness(intersection.uv);
		float lambda = metalness * (1 - stdBRDF::Alpha(roughness)); // 0 - 1
		float p_mat = 1 / (2 - lambda); // 0.5 - 1
		float pd_mat, pd_env;
		vecf3 wi;
		rgbf Le;

		auto env_h = env_light->texture->img->height;
		auto env_w = env_light->texture->img->width;

		if (rand01<float>() < p_mat)
		{
			tie(wi, pd_mat) = SampleBRDF(intersection, wo);
			Le = env_light->Radiance(wi);
			//pd_env = env_light->PDF(wi); // TODO: use your PDF
			/*****************************/
			auto env_texCoor = wi.normalize().cast_to<normalf>().to_sphere_texcoord();
			int j = max(int(env_texCoor[0] * (env_w - 1)), 0);
			int i = max(int(env_texCoor[1] * (env_h - 1)), 0);

			auto sin = (wi.normalize().cross(intersection.n.cast_to<vecf3>())).norm();
			pd_env = 1;
			if (fabs(sin - 0.0) > 1e-4)
			{
				if (env_light_pdf.count({ i,j }) == 0)
				{
					std::cout << "error" << std::endl;
					std::cout << "i:" << i << " j:" << j << std::endl;
					std::cout << "env_h:" << env_h << " env_w:" << env_w << std::endl;
				}
				pd_env = min(env_light_pdf.at({ i, j }) / fabs(sin) / (2 * PI * PI), 1.0); //
			}
			/*****************************/
		}
		else
		{
			//tie(Le, wi, pd_env) = env_light->Sample(intersection.n); // TODO: use your sampling method
			/*****************************/
			auto rand_n = int(rand01<float>() * (env_h * env_w - 1));
			auto rand_p = rand01<float>();
			std::pair<int, int> sample_idx;
			if (rand_p <= std::get<0>(env_light_alias_table_p[rand_n]))
			{
				sample_idx = std::get<0>(env_light_alias_table_idx[rand_n]);
			}
			else sample_idx = std::get<1>(env_light_alias_table_idx[rand_n]);

			auto i = std::get<0>(sample_idx);
			auto j = std::get<1>(sample_idx);

			auto v = float(i) / float(env_h);
			auto u = float(j) / float(env_w);

			float theta = PI * (1 - v);
			float phi = 2 * PI * u;
			auto wi = vecf3(sin(theta) * sin(phi), cos(theta), sin(theta) * cos(phi));

			//Le = env_light->texture->img->At(j, i).to_rgb(); 
			Le = env_light->Radiance(pointf2(u, v));

			auto sin = (wi.cross(intersection.n.cast_to<vecf3>())).norm();
			pd_env = 1;
			if (fabs(sin - 0.0) > 1e-4)
			{
				if (env_light_pdf.count(sample_idx) == 0)
				{
					std::cout << "error" << std::endl;
					std::cout << "i:" << i << " j:" << j << std::endl;
					std::cout << "env_h:" << env_h << " env_w:" << env_w << std::endl;
				}
				pd_env = min(env_light_pdf.at(sample_idx) / fabs(sin) / (2 * PI * PI), 1.0); //
			}
			///*****************************/

			matf3 surface_to_world = svecf::TBN(intersection.n.cast_to<vecf3>(), intersection.tangent);
			matf3 world_to_surface = surface_to_world.inverse();
			svecf s_wo = (world_to_surface * wo).cast_to<svecf>();
			svecf s_wi = (world_to_surface * wi).cast_to<svecf>();
			rgbf albedo = brdf->Albedo(intersection.uv);
			pd_mat = brdf->PDF(albedo, metalness, roughness, s_wi, s_wo);
		}

		rst.L = Le;
		rst.n = -wi.cast_to<normalf>();
		rst.pd = p_mat * pd_mat + (1 - p_mat) * pd_env;
		rst.x = pointf3{ std::numeric_limits<float>::max() };
	}
	return rst;
}

std::tuple<vecf3, float> PathTracer::SampleBRDF(IntersectorClosest::Rst intersection, const vecf3& wo) {
	auto mat = intersection.sobj->Get<Cmpt::Material>();
	if (!mat) return { vecf3{0.f}, 0.f };
	auto brdf = dynamic_cast<const stdBRDF*>(mat->material.get());
	if (!brdf) return { vecf3{0.f}, 0.f };

	matf3 surface_to_world = svecf::TBN(intersection.n.cast_to<vecf3>(), intersection.tangent);
	matf3 world_to_surface = surface_to_world.inverse();
	svecf s_wo = (world_to_surface * wo).cast_to<svecf>();

	rgbf albedo = brdf->Albedo(intersection.uv);
	float metalness = brdf->Metalness(intersection.uv);
	float roughness = brdf->Roughness(intersection.uv);

	auto [s_wi, pdf] = brdf->Sample(albedo, metalness, roughness, s_wo);
	vecf3 wi = surface_to_world * s_wi;

	return { wi,pdf };
}

rgbf PathTracer::BRDF(IntersectorClosest::Rst intersection, const vecf3& wi, const vecf3& wo) {
	auto mat = intersection.sobj->Get<Cmpt::Material>();
	if (!mat) return rgbf{ 1.f,0.f,1.f };
	auto brdf = dynamic_cast<const stdBRDF*>(mat->material.get());
	if (!brdf) return rgbf{ 1.f,0.f,1.f };

	matf3 surface_to_world = svecf::TBN(intersection.n.cast_to<vecf3>(), intersection.tangent);
	matf3 world_to_surface = surface_to_world.inverse();
	svecf s_wi = (world_to_surface * wi).cast_to<svecf>();
	svecf s_wo = (world_to_surface * wo).cast_to<svecf>();

	rgbf albedo = brdf->Albedo(intersection.uv);
	float metalness = brdf->Metalness(intersection.uv);
	float roughness = brdf->Roughness(intersection.uv);

	return brdf->BRDF(albedo, metalness, roughness, s_wi, s_wo);
}
