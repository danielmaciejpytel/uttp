#pragma once
// Own offline-only interpreter. Program parameters/assets remain local Build data.
// No game types, hooks, allocation, file I/O or captured-helper fallback.
#include <array>
#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <emmintrin.h>

namespace utpp::mm::deformoffline {
inline constexpr uint32_t kMaxNodes=512, kMaxOps=1024, kMaxInputs=8, kMaxChannels=32;
inline constexpr uint32_t kMaxTracks=32; // private scalar registers, reset every Evaluate
struct Vec3 { float x=0,y=0,z=0; };
struct Quat { float x=0,y=0,z=0,w=1; };
struct Transform { Vec3 t{}; Quat q{}; Vec3 s{1,1,1}; };
enum class NodeKind:uint32_t { Reference=0,Core=1,Logical=2 };
enum class OpKind:uint32_t { Init=0,Set=1,Point=2,Orient=3,AimObject=4,AimRotation=5,Spline=6,Twist=7,Bounce=8,Limit=9,TrackBounce=10,SplineTrack=11 };
struct Node { int32_t parent=-1; NodeKind kind=NodeKind::Reference; Transform refLocal{}; };
struct WeightedInput { int32_t node=-1; float weight=0; };
// kind: 0..2 PosX/Y/Z, 3..5 ScaleX/Y/Z, 6..8 RotX/Y/Z (degrees).
struct Channel { int32_t target=-1,parent=-1; uint32_t kind=0; float multiplier=1; };
struct Op {
 // TrackBounce: target is a PRIVATE scalar register, not a bone; p[5] is
 // output multiplier. SplineTrack: p[1] names that same-program register.
 // No external/native tracks or previous-call track values are consumed.
 OpKind kind=OpKind::Init; uint32_t sourceHandle=0;
 int32_t target=-1,a=-1,b=-1,c=-1;
 std::array<float,8> p{}; std::array<Vec3,3> v{}; Transform x{};
 uint32_t inputCount=0,channelCount=0;
 std::array<WeightedInput,kMaxInputs> inputs{};
 std::array<Channel,kMaxChannels> channels{};
};
struct Program { uint32_t nodeCount=0,opCount=0; std::array<Node,kMaxNodes> nodes{}; std::array<Op,kMaxOps> ops{}; };
struct Inputs { std::array<Transform,kMaxNodes> model{}; std::array<bool,kMaxNodes> present{}; };
struct Scratch { std::array<Transform,kMaxNodes> model{}; std::array<bool,kMaxNodes> initialized{},written{};
 std::array<float,kMaxTracks> tracks{}; std::array<bool,kMaxTracks> trackReady{}; };
enum class Error:uint32_t { None=0,Bounds,InvalidNode,InvalidOperator,CoreWrite,Uninitialized,InvalidParameter,Nonfinite,Degenerate,MissingCore,UnexpectedInput };
struct Result { Error error=Error::None; uint32_t opIndex=0; int32_t nodeIndex=-1; explicit operator bool()const{return error==Error::None;} };
namespace detail {
// Same result as std::isfinite, as an inline exponent test (the library call is the dominant cost of Valid on the hot path).
inline bool Finite(float x){return (std::bit_cast<uint32_t>(x)&0x7f800000u)!=0x7f800000u;}
inline bool Finite(Vec3 v){return Finite(v.x)&&Finite(v.y)&&Finite(v.z);}
inline bool Finite(Quat q){return Finite(q.x)&&Finite(q.y)&&Finite(q.z)&&Finite(q.w);}
inline Vec3 Add(Vec3 a,Vec3 b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
inline Vec3 Sub(Vec3 a,Vec3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
inline Vec3 Mul(Vec3 a,float f){return {a.x*f,a.y*f,a.z*f};}
inline Vec3 Had(Vec3 a,Vec3 b){return {a.x*b.x,a.y*b.y,a.z*b.z};}
inline float Dot(Vec3 a,Vec3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
inline Vec3 Cross(Vec3 a,Vec3 b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
inline float Dot(Quat a,Quat b){return a.x*b.x+a.y*b.y+a.z*b.z+a.w*b.w;}
inline bool Normalize(Vec3 a,Vec3& out){float n=Dot(a,a);if(!Finite(n)||n<1e-16f)return false;out=Mul(a,1/std::sqrt(n));return Finite(out);}
// A product of unit rotations is unit to a few float ulps, so 1/sqrt(n) around n = 1 is the second-order series (error < 6e-10 for
// |n-1| < 1e-3, below float precision); anything further from unit takes the exact sqrt.
inline bool Normalize(Quat a,Quat& out){
 float n=Dot(a,a);if(!Finite(n)||n<1e-16f)return false;
 const float e=n-1;float f;
 if(e>-1e-3f&&e<1e-3f)f=1+e*(-.5f+e*.375f);else f=1/std::sqrt(n);
 out={a.x*f,a.y*f,a.z*f,a.w*f};return Finite(out);
}
inline Quat Conj(Quat q){return {-q.x,-q.y,-q.z,q.w};}
inline Quat Product(Quat a,Quat b){return {a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w,a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z};}
inline Vec3 Rotate(Quat q,Vec3 v){Vec3 u{q.x,q.y,q.z};return Add(v,Add(Mul(Cross(u,v),2*q.w),Mul(Cross(u,Cross(u,v)),2)));}
// SSE2 forms of the transform algebra used by Compose / Relative. Every lane evaluates exactly the scalar expression of Product,
// Rotate, Had, Sub and Dot above (same operands, same left-to-right order, a - b as a + (-b), no fused or approximate instructions),
// so the results are the same bits as the scalar definitions. Lane 3 of a 3-vector carries a finite filler that is never stored or tested.
namespace sse {
inline __m128 LoadT(const Transform& x){return _mm_loadu_ps(&x.t.x);}                                          // tx ty tz qx
inline __m128 LoadQ(const Transform& x){return _mm_loadu_ps(&x.q.x);}                                          // qx qy qz qw
inline __m128 LoadS(const Transform& x){return _mm_shuffle_ps(_mm_loadu_ps(&x.q.w),_mm_loadu_ps(&x.q.w),_MM_SHUFFLE(0,3,2,1));} // sx sy sz qw
inline __m128 Sign(uint32_t x,uint32_t y,uint32_t z,uint32_t w){return _mm_castsi128_ps(_mm_set_epi32(int(w<<31),int(z<<31),int(y<<31),int(x<<31)));}
template<int I> inline __m128 Splat(__m128 v){return _mm_shuffle_ps(v,v,_MM_SHUFFLE(I,I,I,I));}
// ((a.w*b + s2*(a.x*b.wzyx)) + s3*(a.y*b.zwxy)) + s4*(a.z*b.yxwz), lane by lane the scalar Product.
inline __m128 Product(__m128 a,__m128 b){
 const __m128 t1=_mm_mul_ps(Splat<3>(a),b);
 const __m128 t2=_mm_xor_ps(_mm_mul_ps(Splat<0>(a),_mm_shuffle_ps(b,b,_MM_SHUFFLE(0,1,2,3))),Sign(0,1,0,1));
 const __m128 t3=_mm_xor_ps(_mm_mul_ps(Splat<1>(a),_mm_shuffle_ps(b,b,_MM_SHUFFLE(1,0,3,2))),Sign(0,0,1,1));
 const __m128 t4=_mm_xor_ps(_mm_mul_ps(Splat<2>(a),_mm_shuffle_ps(b,b,_MM_SHUFFLE(2,3,0,1))),Sign(1,0,0,1));
 return _mm_add_ps(_mm_add_ps(_mm_add_ps(t1,t2),t3),t4);
}
// a.yzx * b.zxy - a.zxy * b.yzx (lane 3 = filler).
inline __m128 Cross(__m128 a,__m128 b){
 const __m128 ay=_mm_shuffle_ps(a,a,_MM_SHUFFLE(3,0,2,1)),bz=_mm_shuffle_ps(b,b,_MM_SHUFFLE(3,1,0,2));
 const __m128 az=_mm_shuffle_ps(a,a,_MM_SHUFFLE(3,1,0,2)),by=_mm_shuffle_ps(b,b,_MM_SHUFFLE(3,0,2,1));
 return _mm_sub_ps(_mm_mul_ps(ay,bz),_mm_mul_ps(az,by));
}
// v + (Cross(u,v)*(2*q.w) + Cross(u,Cross(u,v))*2); u = q.xyz (lane 3 of q is never read by Cross into lanes 0..2).
inline __m128 Rotate(__m128 q,__m128 v){
 const __m128 c1=Cross(q,v),c2=Cross(q,c1);
 const __m128 w2=_mm_set1_ps(2*_mm_cvtss_f32(Splat<3>(q)));
 return _mm_add_ps(v,_mm_add_ps(_mm_mul_ps(c1,w2),_mm_mul_ps(c2,_mm_set1_ps(2.f))));
}
// ((x*x + y*y) + z*z) + w*w: the scalar Dot of a quaternion with itself.
inline float Dot4(__m128 a){
 const __m128 p=_mm_mul_ps(a,a);
 __m128 s=_mm_add_ss(p,_mm_shuffle_ps(p,p,_MM_SHUFFLE(1,1,1,1)));
 s=_mm_add_ss(s,_mm_shuffle_ps(p,p,_MM_SHUFFLE(2,2,2,2)));
 return _mm_cvtss_f32(_mm_add_ss(s,_mm_shuffle_ps(p,p,_MM_SHUFFLE(3,3,3,3))));
}
// Finite (exponent not all ones) for the lanes selected by mask (bit k = lane k).
inline bool Finite(__m128 v,int mask){
 const __m128i e=_mm_set1_epi32(0x7f800000);
 const __m128i hit=_mm_cmpeq_epi32(_mm_and_si128(_mm_castps_si128(v),e),e);
 return (_mm_movemask_ps(_mm_castsi128_ps(hit))&mask)==0;
}
// Scalar Normalize(Quat) on a vector: same n, same branch and factor.
inline bool Normalize(__m128 a,__m128& out){
 const float n=Dot4(a);if(!detail::Finite(n)||n<1e-16f)return false;
 const float e=n-1;float f;
 if(e>-1e-3f&&e<1e-3f)f=1+e*(-.5f+e*.375f);else f=1/std::sqrt(n);
 out=_mm_mul_ps(a,_mm_set1_ps(f));return true;
}
inline void Store(Transform& out,__m128 t,__m128 q,__m128 s){
 _mm_storel_pi(reinterpret_cast<__m64*>(&out.t.x),t);_mm_store_ss(&out.t.z,_mm_movehl_ps(t,t));
 _mm_storeu_ps(&out.q.x,q);
 _mm_storel_pi(reinterpret_cast<__m64*>(&out.s.x),s);_mm_store_ss(&out.s.z,_mm_movehl_ps(s,s));
}
// Valid() of (t, q, s): the q dot in scalar order, finite t / q / s / n, unit within 1e-3.
inline bool Valid(__m128 t,__m128 q,__m128 s){
 const float n=Dot4(q);
 return Finite(t,7)&&Finite(q,15)&&Finite(s,7)&&detail::Finite(n)&&std::fabs(n-1.f)<1e-3f;
}
} // namespace sse
// Same predicate as: n = Dot(x.q,x.q); Finite(x.t) && Finite(x.q) && Finite(x.s) && Finite(n) && |n - 1| < 1e-3.
inline bool Valid(const Transform& x){return sse::Valid(sse::LoadT(x),sse::LoadQ(x),sse::LoadS(x));}
// Same results and the same writes to `out` as: Normalize(Product(a.q,b.q),out.q) (fails before any write when the product is degenerate),
// then out.t=Add(a.t,Rotate(a.q,Had(a.s,b.t))), out.s=Had(a.s,b.s), return Valid(out).
inline bool Compose(const Transform& a,const Transform& b,Transform& out){
 const __m128 aq=sse::LoadQ(a),as=sse::LoadS(a),at=sse::LoadT(a),bt=sse::LoadT(b),bs=sse::LoadS(b);
 __m128 q;if(!sse::Normalize(sse::Product(aq,sse::LoadQ(b)),q))return false;
 if(!sse::Finite(q,15)){_mm_storeu_ps(&out.q.x,q);return false;}
 const __m128 t=_mm_add_ps(at,sse::Rotate(aq,_mm_mul_ps(as,bt))),s=_mm_mul_ps(as,bs);
 sse::Store(out,t,q,s);return sse::Valid(t,q,s);
}
// Same results and writes as: scale guard (no write), out.t=Rotate(Conj(a.q),Sub(b.t,a.t))/a.s, then Normalize(Product(Conj(a.q),b.q),out.q),
// out.s=b.s/a.s, return Valid(out).
inline bool Relative(const Transform& a,const Transform& b,Transform& out){
 if(std::fabs(a.s.x)<1e-8f||std::fabs(a.s.y)<1e-8f||std::fabs(a.s.z)<1e-8f)return false;
 const __m128 aq=_mm_xor_ps(sse::LoadQ(a),sse::Sign(1,1,1,0)),bq=sse::LoadQ(b),bs=sse::LoadS(b);
 const __m128 safe=_mm_or_ps(_mm_and_ps(sse::LoadS(a),_mm_castsi128_ps(_mm_set_epi32(0,-1,-1,-1))),_mm_set_ps(1.f,0,0,0)); // lane 3 = 1 (filler)
 const __m128 t=_mm_div_ps(sse::Rotate(aq,_mm_sub_ps(sse::LoadT(b),sse::LoadT(a))),safe);
 _mm_storel_pi(reinterpret_cast<__m64*>(&out.t.x),t);_mm_store_ss(&out.t.z,_mm_movehl_ps(t,t));
 __m128 q;if(!sse::Normalize(sse::Product(aq,bq),q))return false;
 _mm_storeu_ps(&out.q.x,q);if(!sse::Finite(q,15))return false;
 const __m128 s=_mm_div_ps(bs,safe);
 _mm_storel_pi(reinterpret_cast<__m64*>(&out.s.x),s);_mm_store_ss(&out.s.z,_mm_movehl_ps(s,s));
 return sse::Valid(t,q,s);
}
inline bool Slerp(Quat a,Quat b,float u,Quat& out){
 float dot=Dot(a,b);if(dot<0){b={-b.x,-b.y,-b.z,-b.w};dot=-dot;}
 dot=std::clamp(dot,-1.f,1.f);float aa=1-u,bb=u;
 if(dot<.9995f){float theta=std::acos(dot),den=std::sin(theta);if(std::fabs(den)<1e-8f)return false;aa=std::sin((1-u)*theta)/den;bb=std::sin(u*theta)/den;}
 return Normalize({a.x*aa+b.x*bb,a.y*aa+b.y*bb,a.z*aa+b.z*bb,a.w*aa+b.w*bb},out);
}
inline bool Shortest(Vec3 from,Vec3 to,Quat& q){
 Vec3 a,b;if(!Normalize(from,a)||!Normalize(to,b))return false;float d=Dot(a,b);if(d<-.999999f)return false;Vec3 v=Cross(a,b);return Normalize({v.x,v.y,v.z,1+d},q);
}
inline bool AxisAngle(Vec3 axis,float angle,Quat& q){Vec3 n;if(!Normalize(axis,n)||!Finite(angle))return false;float s=std::sin(angle*.5f);q={n.x*s,n.y*s,n.z*s,std::cos(angle*.5f)};return Normalize(q,q);}
inline bool Basis(Vec3 forward,Vec3 up,std::array<Vec3,3>& b){
 if(!Normalize(forward,b[0])||!Normalize(Sub(up,Mul(b[0],Dot(up,b[0]))),b[1]))return false;b[2]=Cross(b[0],b[1]);return Finite(b[2]);
}
inline bool MatrixQuat(const float m[3][3],Quat& q){
 float trace=m[0][0]+m[1][1]+m[2][2];
 if(trace>0){float s=std::sqrt(trace+1)*2;q={(m[2][1]-m[1][2])/s,(m[0][2]-m[2][0])/s,(m[1][0]-m[0][1])/s,s*.25f};}
 else if(m[0][0]>m[1][1]&&m[0][0]>m[2][2]){float s=std::sqrt(1+m[0][0]-m[1][1]-m[2][2])*2;q={s*.25f,(m[0][1]+m[1][0])/s,(m[0][2]+m[2][0])/s,(m[2][1]-m[1][2])/s};}
 else if(m[1][1]>m[2][2]){float s=std::sqrt(1+m[1][1]-m[0][0]-m[2][2])*2;q={(m[0][1]+m[1][0])/s,s*.25f,(m[1][2]+m[2][1])/s,(m[0][2]-m[2][0])/s};}
 else {float s=std::sqrt(1+m[2][2]-m[0][0]-m[1][1])*2;q={(m[0][2]+m[2][0])/s,(m[1][2]+m[2][1])/s,s*.25f,(m[1][0]-m[0][1])/s};}
 return Normalize(q,q);
}
inline bool Aim(Vec3 direction,Vec3 up,Vec3 localForward,Vec3 localUp,Quat& q){
 std::array<Vec3,3> world,local;if(!Basis(direction,up,world)||!Basis(localForward,localUp,local))return false;
 const float W[3][3]={{world[0].x,world[1].x,world[2].x},{world[0].y,world[1].y,world[2].y},{world[0].z,world[1].z,world[2].z}};
 const float L[3][3]={{local[0].x,local[1].x,local[2].x},{local[0].y,local[1].y,local[2].y},{local[0].z,local[1].z,local[2].z}};
 float M[3][3]{};for(int r=0;r<3;++r)for(int c=0;c<3;++c)for(int k=0;k<3;++k)M[r][c]+=W[r][k]*L[c][k];return MatrixQuat(M,q);
}
// Rotation channel of a bounce output (RotX/RotY/RotZ, degrees). The local rotation is split as R = Rz(yaw) * Rx(pitch) * Ry(roll)
// (column vectors; RotX = pitch, RotY = roll, RotZ = yaw), the chosen angle is replaced and the rotation rebuilt. Same convention and
// units as the deformer graph (tools/analysis/deformer_rotation_channels.py, confirmed on the engine's recorded elbow helpers). Fails
// closed within 0.06 degrees of the pitch gimbal boundary (+-90), where the split is not unique.
inline bool ReplaceEuler(Quat q,uint32_t axis,float degrees,Quat& out){
 const double x=q.x,y=q.y,z=q.z,w=q.w;
 const double r21=2*(y*z+x*w),r01=2*(x*y-z*w),r11=1-2*(x*x+z*z),r20=2*(x*z-y*w),r22=1-2*(x*x+y*y);
 double pitch=std::asin(std::clamp(r21,-1.0,1.0));
 if(std::fabs(std::cos(pitch))<1e-3||!Finite(degrees))return false; // float rotations carry ~1e-7: below this the yaw / roll split is noise
 double yaw=std::atan2(-r01,r11),roll=std::atan2(-r20,r22);
 const double v=double(degrees)*(3.14159265358979323846/180.0);
 if(axis==0)pitch=v;else if(axis==1)roll=v;else yaw=v;
 const double cp=std::cos(pitch),sp=std::sin(pitch),cb=std::cos(roll),sb=std::sin(roll),cy=std::cos(yaw),sy=std::sin(yaw);
 // Rz(yaw) * Rx(pitch) * Ry(roll)
 const double zx[3][3]={{cy,-sy*cp,sy*sp},{sy,cy*cp,-cy*sp},{0,sp,cp}};
 const double ry[3][3]={{cb,0,sb},{0,1,0},{-sb,0,cb}};
 float m[3][3]{};
 for(int i=0;i<3;++i)for(int j=0;j<3;++j){double t=0;for(int k=0;k<3;++k)t+=zx[i][k]*ry[k][j];m[i][j]=float(t);}
 return MatrixQuat(m,out);
}
inline float Component(Vec3 v,uint32_t a){return a==0?v.x:a==1?v.y:v.z;}
inline void SetComponent(Vec3& v,uint32_t a,float f){if(a==0)v.x=f;else if(a==1)v.y=f;else v.z=f;}
inline bool Index(int32_t n,uint32_t count){return n>=0&&uint32_t(n)<count;}
inline bool Integer(float f,int lo,int hi){return Finite(f)&&f>=float(lo)&&f<=float(hi)&&std::floor(f)==f;}
} // detail

// Cold validation: no unsupported operators, dynamic tracks, core writes or
// uninitialized reads. Exporter flattens source graph pose links in proven order.
inline Result Validate(const Program& program){
 using namespace detail;Result result{};
 if(program.nodeCount==0||program.nodeCount>kMaxNodes||program.opCount==0||program.opCount>kMaxOps)return {Error::Bounds};
 std::array<bool,kMaxNodes> ready{};
 std::array<bool,kMaxTracks> trackReady{};
 for(uint32_t n=0;n<program.nodeCount;++n){const Node& node=program.nodes[n];
  if(uint32_t(node.kind)>2||!Valid(node.refLocal))return {Error::InvalidNode,0,int32_t(n)};
  if(node.kind==NodeKind::Logical)continue;
  if(node.parent>=0&&(!Index(node.parent,n)||!ready[node.parent]))return {Error::InvalidNode,0,int32_t(n)};
  ready[n]=true;
 }
 for(uint32_t oi=0;oi<program.opCount;++oi){const Op& op=program.ops[oi];
  auto read=[&](int32_t i){return Index(i,program.nodeCount)&&ready[i];};
  auto write=[&](int32_t i){return Index(i,program.nodeCount)&&program.nodes[i].kind!=NodeKind::Core;};
  auto fail=[&](Error e,int32_t n=-1){return Result{e,oi,n};};
  if(uint32_t(op.kind)>11)return fail(Error::InvalidOperator);
  for(float f:op.p)if(!Finite(f))return fail(Error::Nonfinite);
  for(Vec3 v:op.v)if(!Finite(v))return fail(Error::Nonfinite);
  if(!Valid(op.x)||op.inputCount>kMaxInputs||op.channelCount>kMaxChannels)return fail(Error::InvalidParameter);
  if(op.kind==OpKind::TrackBounce){
   if(!read(op.a)||!read(op.b)||!Index(op.target,kMaxTracks)||op.p[2]<0||op.p[3]<0)return fail(Error::InvalidParameter);
   trackReady[op.target]=true;continue;
  }
  if(op.kind==OpKind::Bounce){
   if(!read(op.a)||!read(op.b)||op.channelCount==0||op.p[2]<0||op.p[3]<0)return fail(Error::InvalidParameter);
   for(uint32_t k=0;k<op.channelCount;++k){const Channel& c=op.channels[k];
    if(!write(c.target))return fail(Error::CoreWrite,c.target);
    if(!read(c.target)||!read(c.parent)||c.kind>8||!Finite(c.multiplier))return fail(Error::InvalidParameter,c.target);
    // Driver must be sampled once before channel writes; input alias is allowed
    // because Evaluate caches its scalar before touching any target.
    ready[c.target]=true;
   }
   continue;
  }
  if(!write(op.target))return fail(Error::CoreWrite,op.target);
  if(op.kind==OpKind::Init){
   if(program.nodes[op.target].kind!=NodeKind::Logical||ready[op.target]||!read(op.a))return fail(Error::InvalidParameter,op.target);
  } else {
   if(!read(op.target))return fail(Error::Uninitialized,op.target);
   switch(op.kind){
   case OpKind::Set:
    if(!Integer(op.p[0],0,3)||(int(op.p[0])!=3&&!read(op.a))||(op.b!=-1&&!read(op.b)))return fail(Error::InvalidParameter);break;
   case OpKind::Point:case OpKind::Orient:
    if(op.inputCount==0)return fail(Error::InvalidParameter);
    for(uint32_t k=0;k<op.inputCount;++k)if(!read(op.inputs[k].node)||!Finite(op.inputs[k].weight)||op.inputs[k].weight<0||op.inputs[k].weight>1)return fail(Error::InvalidParameter);
    if(op.kind==OpKind::Point){float sum=0;for(uint32_t k=0;k<op.inputCount;++k)sum+=op.inputs[k].weight;if(std::fabs(sum-1)>1e-4f)return fail(Error::InvalidParameter);}
    break;
   case OpKind::AimObject:case OpKind::AimRotation:
    if(!read(op.a)||!read(op.b))return fail(Error::Uninitialized);break;
   case OpKind::Spline:
    if(!read(op.a)||!read(op.b)||!read(op.c)||op.p[0]<0||op.p[0]>1)return fail(Error::InvalidParameter);break;
   case OpKind::SplineTrack:
    if(!read(op.a)||!read(op.b)||!read(op.c)||!Integer(op.p[1],0,int(kMaxTracks)-1)||!trackReady[uint32_t(op.p[1])])return fail(Error::InvalidParameter);break;
   case OpKind::Twist:
    if(!read(op.a)||!read(op.b)||!read(op.c))return fail(Error::Uninitialized);break;
   case OpKind::Limit:
    if(!read(op.a)||!Integer(op.p[0],0,7)||!Integer(op.p[1],0,7))return fail(Error::InvalidParameter);break;
   default:return fail(Error::InvalidOperator);
   }
  }
  ready[op.target]=true;
 }
 return result;
}

// Requires a previously validated immutable Program. Scratch is caller-owned.
// On failure its candidate data must be discarded; input core is never changed.
inline Result Evaluate(const Program& program,const Inputs& input,Scratch& scratch){
 using namespace detail;
 if(program.nodeCount==0||program.nodeCount>kMaxNodes||program.opCount>kMaxOps)return {Error::Bounds};
 scratch.initialized.fill(false);scratch.written.fill(false);
 scratch.trackReady.fill(false);scratch.tracks.fill(0);
 for(uint32_t n=0;n<program.nodeCount;++n){const Node& node=program.nodes[n];
  if(input.present[n]&&node.kind!=NodeKind::Core)return {Error::UnexpectedInput,0,int32_t(n)};
  if(node.kind==NodeKind::Logical)continue;
  if(node.kind==NodeKind::Core){
   if(!input.present[n])return {Error::MissingCore,0,int32_t(n)};
   if(!Valid(input.model[n]))return {Error::Nonfinite,0,int32_t(n)};
   scratch.model[n]=input.model[n]; // Preserve actual core input bit for bit.
  } else {
   Transform parent{};if(node.parent>=0){if(!Index(node.parent,n)||!scratch.initialized[node.parent])return {Error::Uninitialized,0,int32_t(n)};parent=scratch.model[node.parent];}
   if(!Compose(parent,node.refLocal,scratch.model[n]))return {Error::Degenerate,0,int32_t(n)};
  }
  scratch.initialized[n]=true;
 }
 for(uint32_t oi=0;oi<program.opCount;++oi){const Op& op=program.ops[oi];
  auto fail=[&](Error error,int32_t n=-1){return Result{error,oi,n};};
  auto ready=[&](int32_t n){return Index(n,program.nodeCount)&&scratch.initialized[n];};
  auto allowed=[&](int32_t n){return Index(n,program.nodeCount)&&program.nodes[n].kind!=NodeKind::Core;};
  auto commit=[&](int32_t n,const Transform& t){scratch.model[n]=t;scratch.initialized[n]=true;scratch.written[n]=true;};
  auto store=[&](int32_t n,const Transform& t){if(!allowed(n)||!Valid(t))return false;commit(n,t);return true;};
  if(op.kind==OpKind::Bounce||op.kind==OpKind::TrackBounce){
   if(!ready(op.a)||!ready(op.b)||op.channelCount>kMaxChannels)return fail(Error::Uninitialized);
   Transform relative;if(!Relative(scratch.model[op.a],scratch.model[op.b],relative))return fail(Error::Degenerate);
   float value=relative.t.x;float multiplier=value<0?op.p[1]:op.p[0];float d=std::max(0.f,std::fabs(value)-op.p[2]);float scalar=(op.p[3]>0&&d<op.p[3]?d*d/op.p[3]:d)*multiplier+op.p[4];
   if(!Finite(scalar))return fail(Error::Nonfinite);
   if(op.kind==OpKind::TrackBounce){
    if(!Index(op.target,kMaxTracks)||!Finite(op.p[5])||!Finite(scalar*op.p[5]))return fail(Error::InvalidParameter);
    scratch.tracks[op.target]=scalar*op.p[5];scratch.trackReady[op.target]=true;continue;
   }
   // Consecutive channels of one target with the same parent (one bounce output) share a single local/compose round trip.
   for(uint32_t k=0;k<op.channelCount;){const Channel& channel=op.channels[k];
    if(!allowed(channel.target))return fail(Error::CoreWrite,channel.target);
    if(!ready(channel.target)||!ready(channel.parent))return fail(Error::InvalidParameter);
    uint32_t end=k+1;while(end<op.channelCount&&op.channels[end].target==channel.target&&op.channels[end].parent==channel.parent)++end;
    const Transform parent=scratch.model[channel.parent];Transform local,out;
    if(!Relative(parent,scratch.model[channel.target],local))return fail(Error::Degenerate);
    for(;k<end;++k){const Channel& c=op.channels[k];
     if(c.kind>8)return fail(Error::InvalidParameter);
     if(c.kind<3)SetComponent(local.t,c.kind,scalar*c.multiplier);else if(c.kind<6)SetComponent(local.s,c.kind-3,scalar*c.multiplier);
     else if(!ReplaceEuler(local.q,c.kind-6,scalar*c.multiplier,local.q))return fail(Error::Degenerate,channel.target);
    }
    if(!Compose(parent,local,out))return fail(Error::Degenerate,channel.target);
    commit(channel.target,out); // Compose already proved Valid(out); channel.target passed allowed() above
   }
   continue;
  }
  if(!allowed(op.target))return fail(Error::CoreWrite,op.target);
  Transform result=ready(op.target)?scratch.model[op.target]:Transform{};
  if(op.kind!=OpKind::Init&&!ready(op.target))return fail(Error::Uninitialized,op.target);
  switch(op.kind){
  case OpKind::Init:
   if(program.nodes[op.target].kind!=NodeKind::Logical||ready(op.target)||!ready(op.a))return fail(Error::InvalidParameter);
   if(!Compose(scratch.model[op.a],op.x,result))return fail(Error::Degenerate);
   commit(op.target,result);continue; // Compose already proved Valid(result); op.target passed allowed() above
  case OpKind::Set:{
   int method=int(op.p[0]);if(!Integer(op.p[0],0,3))return fail(Error::InvalidParameter);
   if(method!=3){if(!ready(op.a))return fail(Error::Uninitialized);const Transform source=scratch.model[op.a];if(method==0)result=source;else if(method==1)result.t=source.t;else result.q=source.q;}
   if(op.b!=-1){if(!ready(op.b))return fail(Error::Uninitialized);
    // If target is its own offset space, native snapping has already updated it.
    const Transform parent=op.b==op.target?result:scratch.model[op.b];Transform local;
    if(!Relative(parent,result,local))return fail(Error::Degenerate);
    local.t=Add(local.t,op.x.t);local.s=Had(op.x.s,local.s);if(!Normalize(Product(op.x.q,local.q),local.q)||!Compose(parent,local,result))return fail(Error::Degenerate);
   }break;}
  case OpKind::Point:
   if(op.inputCount==0||op.inputCount>kMaxInputs)return fail(Error::InvalidParameter);
   result.t={};for(uint32_t k=0;k<op.inputCount;++k){if(!ready(op.inputs[k].node))return fail(Error::Uninitialized);result.t=Add(result.t,Mul(scratch.model[op.inputs[k].node].t,op.inputs[k].weight));}break;
  case OpKind::Orient:
   if(op.inputCount==0||op.inputCount>kMaxInputs||!ready(op.inputs[0].node))return fail(Error::InvalidParameter);
   result.q=scratch.model[op.inputs[0].node].q;
   for(uint32_t k=1;k<op.inputCount;++k){if(!ready(op.inputs[k].node)||!Slerp(result.q,scratch.model[op.inputs[k].node].q,op.inputs[k].weight,result.q))return fail(Error::Degenerate);}break;
  case OpKind::AimObject:case OpKind::AimRotation:{
   if(!ready(op.a)||!ready(op.b))return fail(Error::Uninitialized);
   const Transform target=scratch.model[op.a],up=scratch.model[op.b];Vec3 upVector=op.kind==OpKind::AimRotation?Rotate(up.q,op.v[2]):Sub(up.t,result.t);
   if(!Aim(Sub(target.t,result.t),upVector,op.v[0],op.v[1],result.q))return fail(Error::Degenerate);break;}
  case OpKind::Spline:case OpKind::SplineTrack:{
   if(!ready(op.a)||!ready(op.b)||!ready(op.c))return fail(Error::Uninitialized);float u=op.p[0];
   if(op.kind==OpKind::SplineTrack){
    if(!Integer(op.p[1],0,int(kMaxTracks)-1)||!scratch.trackReady[uint32_t(op.p[1])])return fail(Error::Uninitialized);
    u=std::clamp(scratch.tracks[uint32_t(op.p[1])],0.f,1.f);
   }
   if(u<0||u>1||!Finite(u))return fail(Error::InvalidParameter);
   Vec3 ab=Add(Mul(scratch.model[op.a].t,1-u),Mul(scratch.model[op.b].t,u));Vec3 bc=Add(Mul(scratch.model[op.b].t,1-u),Mul(scratch.model[op.c].t,u));result.t=Add(Mul(ab,1-u),Mul(bc,u));break;}
  case OpKind::Twist:{
   if(!ready(op.a)||!ready(op.b)||!ready(op.c))return fail(Error::Uninitialized);
   Quat A=scratch.model[op.a].q,B=scratch.model[op.b].q,align,rotation;
   Vec3 af=Rotate(A,op.v[0]),aside=Rotate(A,op.v[1]);if(!Shortest(Rotate(B,op.v[0]),af,align))return fail(Error::Degenerate);
   Vec3 bside=Rotate(Product(align,B),op.v[1]);Vec3 aUnit,bUnit;if(!Normalize(aside,aUnit)||!Normalize(bside,bUnit))return fail(Error::Degenerate);
   float angle=std::acos(std::clamp(Dot(aUnit,bUnit),-1.f,1.f));if(Dot(bUnit,Cross(af,aUnit))<0)angle=-angle;
   float scale=angle<0?op.p[1]:op.p[0];if(!AxisAngle(op.v[2],angle*scale,rotation)||!Normalize(Product(scratch.model[op.c].q,rotation),result.q))return fail(Error::Degenerate);break;}
  case OpKind::Limit:{
   if(!ready(op.a)||!Integer(op.p[0],0,7)||!Integer(op.p[1],0,7))return fail(Error::InvalidParameter);Transform parent=scratch.model[op.a],local;
   if(!Relative(parent,result,local))return fail(Error::Degenerate);uint32_t mn=uint32_t(op.p[0]),mx=uint32_t(op.p[1]);
   for(uint32_t a=0;a<3;++a){float value=Component(local.t,a);if(mn&(1u<<a))value=std::max(value,Component(op.v[0],a));if(mx&(1u<<a))value=std::min(value,Component(op.v[1],a));SetComponent(local.t,a,value);}
   if(!Compose(parent,local,result))return fail(Error::Degenerate);break;}
  default:return fail(Error::InvalidOperator);
  }
  if(!store(op.target,result))return fail(Error::Nonfinite,op.target);
 }
 return {};
}
} // namespace utpp::mm::deformoffline
