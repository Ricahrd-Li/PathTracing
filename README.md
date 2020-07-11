# Path Tracing

**The codes of render engine and basic libs are from teaching assistant Ubpa.**

Build Method
+ use cmake: config, generate, then build. 

Editor Interface
![](./images/interface.PNG)

For core code of path tracing, please see: 
+ [header](https://github.com/Ricahrd-Li/PathTracing/blob/master/src/PathTracer/PathTracer.h)
+ [cpp](https://github.com/Ricahrd-Li/PathTracing/blob/master/src/PathTracer/PathTracer.cpp)

For introduction, please see this Zhihu article: [link](https://zhuanlan.zhihu.com/p/138317358)

**Note:**
if you encounter error at runtime about invalid memory access, you probably need to add the codes below:
```cpp
#ifdef GL_SAMPLER_BINDING
#undef GL_SAMPLER_BINDING
#endif
```
to file ``imgui_impl_opengl3.cpp`` at line 143. This should fix the error. 