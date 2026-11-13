// Metamorphic checks against physical feature vectors.
#include "../src/mm/MotionPlayRate.hpp"
#include <cmath>
#include <cstdio>
#include <limits>
#include <set>
#include <vector>
using namespace utpp::mm;
static int passed=0,failed=0;
static void Check(const char* label,bool ok){std::printf("%s %s\n",ok?"PASS":"FAIL",label);ok?++passed:++failed;}
static bool Near(float a,float b,float e=2e-4f){return std::isfinite(a)&&std::fabs(a-b)<=e;}
static bool Eligible(const motion::ChannelRecord& c){return (c.flags&motion::kChanTrajectory)&&c.kind==2&&!(c.flags&motion::kChanNormalizeOutput);}
static std::vector<float> Raw(const motion::MotionDatabase& db,uint32_t d,uint32_t r){
 const auto& rec=db.Db(d);std::vector<float> q(rec.dims);
 for(uint32_t k=0;k<rec.dims;++k)q[k]=db.Features(d,r)[k]/db.NormScale(rec.norm)[k]+db.NormMean(rec.norm)[k];
 return q;
}
static double Speed(const motion::MotionDatabase& db,uint32_t d,const std::vector<float>& q){
 const auto& s=db.Schema(db.Db(d).schema);double total=0;
 for(uint32_t i=0;i<s.numChannels;++i){const auto& c=db.Channel(s.firstChannel+i);if(!Eligible(c))continue;double square=0;for(uint32_t k=0;k<c.cardinality;++k)square+=double(q[c.dataOffset+k])*q[c.dataOffset+k];total+=std::sqrt(square);}
 return total;
}
int main(int argc,char** argv){
 if(argc!=2){std::fprintf(stderr,"usage: MotionPlayRateTest actual405E.utppmm\n");return 2;}
 motion::MotionDatabase db;std::string error;if(!db.LoadFromFile(argv[1],motion::kRigIdentityWoman,error)){std::fprintf(stderr,"%s\n",error.c_str());return 2;}
 float dummy=0;motion::MotionDatabase empty;
 Check("unloaded database fails safely",Near(rate2::Calculate(empty,0,0,&dummy,.85f,1.15f).wanted,1));
 Check("invalid database index fails safely",Near(rate2::Calculate(db,db.NumDatabases(),0,&dummy,.85f,1.15f).wanted,1));
 std::set<uint32_t> schemas;uint32_t tested=0,multiple=0;bool unequalScale=false,weightedOracleDistinct=false;
 for(uint32_t d=0;d<db.NumDatabases();++d){
  if(!db.DatabaseUsable(d)||!db.Db(d).numRows)continue;
  const auto& rec=db.Db(d);const auto& schema=db.Schema(rec.schema);schemas.insert(rec.schema);
  uint32_t row=0;auto base=Raw(db,d,row);
  for(;row<rec.numRows;++row){base=Raw(db,d,row);if(Speed(db,d,base)>1e-6)break;}
  ++tested;std::printf("DB %u rows %u schema %u movingRow %u\n",d,rec.numRows,rec.schema,row);
  if(row==rec.numRows){base=Raw(db,d,0);const auto zero=rate2::Calculate(db,d,0,base.data(),.85f,1.15f);Check("zero-motion database denominator fallback",Near(zero.ratio,1)&&Near(zero.wanted,1));continue;}
  const auto identity=rate2::Calculate(db,d,row,base.data(),.85f,1.15f);
  Check("invalid local row fails safely",Near(rate2::Calculate(db,d,rec.numRows,base.data(),.85f,1.15f).wanted,1));
  Check("physical selected row identity",identity.valid&&Near(identity.ratio,1)&&Near(identity.wanted,1));
  Check("pose speed independently decoded",Near(identity.poseSpeed,float(Speed(db,d,base)),.02f));
  std::vector<uint32_t> velocities;
  for(uint32_t i=0;i<schema.numChannels;++i){const auto& c=db.Channel(schema.firstChannel+i);if(Eligible(c))velocities.push_back(schema.firstChannel+i);}
  Check("all eligible channels counted",identity.channels==velocities.size());
  for(float factor:{.5f,2.f,0.f}){
   auto q=base;for(auto i:velocities){const auto& c=db.Channel(i);for(uint32_t k=0;k<c.cardinality;++k)q[c.dataOffset+k]*=factor;}
   auto r=rate2::Calculate(db,d,row,q.data(),.85f,1.15f);
   Check("uniform physical velocity scaling and interval clamp",Near(r.ratio,factor)&&Near(r.wanted,factor==2?1.15f:.85f));
  }
  auto ignored=base;for(uint32_t i=0;i<schema.numChannels;++i){const auto& c=db.Channel(schema.firstChannel+i);if(!Eligible(c))for(uint32_t k=0;k<c.cardinality;++k)ignored[c.dataOffset+k]=1234.f+float(k);}
  Check("position heading normalized direction and bone channels ignored",Near(rate2::Calculate(db,d,row,ignored.data(),.85f,1.15f).ratio,1));
  Check("fixed interval compensates trajectory multiplier",Near(rate2::Calculate(db,d,row,base.data(),1.1f,1.1f,2).wanted,.55f));
  Check("fixed interval zero multiplier fallback",Near(rate2::Calculate(db,d,row,base.data(),1.1f,1.1f,0).wanted,1.1f));
  Check("variable interval does not double-apply multiplier",Near(rate2::Calculate(db,d,row,base.data(),.85f,1.15f,2).wanted,1));
  Check("invalid interval fallback",Near(rate2::Calculate(db,d,row,base.data(),2,1).wanted,1)&&Near(rate2::Calculate(db,d,row,base.data(),0,1).wanted,1));
  Check("nonfinite interval fails safely",!rate2::Calculate(db,d,row,base.data(),std::numeric_limits<float>::quiet_NaN(),1).valid);
  if(!velocities.empty()){auto nonfinite=base;nonfinite[db.Channel(velocities.front()).dataOffset]=std::numeric_limits<float>::infinity();const auto bad=rate2::Calculate(db,d,row,nonfinite.data(),.85f,1.15f);Check("nonfinite physical velocity fails safely",!bad.valid&&Near(bad.wanted,1));}
  Check("missing query fallback is unclamped one",Near(rate2::Calculate(db,d,row,nullptr,1.2f,1.5f).wanted,1));
  if(velocities.size()>1){
   ++multiple;auto q=base;double expectedTotal=0,weightedNumerator=0,weightedDenominator=0;
   for(size_t n=0;n<velocities.size();++n){const auto& c=db.Channel(velocities[n]);const float f=n==0?.25f:1.75f;double mag2=0,wm2=0;for(uint32_t k=0;k<c.cardinality;++k){const auto o=c.dataOffset+k;mag2+=double(base[o])*base[o];wm2+=double(base[o]*db.NormScale(rec.norm)[o])*(base[o]*db.NormScale(rec.norm)[o]);q[o]*=f;if(k&&db.NormScale(rec.norm)[o]!=db.NormScale(rec.norm)[c.dataOffset])unequalScale=true;}expectedTotal+=std::sqrt(mag2)*f;weightedNumerator+=std::sqrt(wm2)*f;weightedDenominator+=std::sqrt(wm2);}
   const float expected=float(expectedTotal/Speed(db,d,base));const auto result=rate2::Calculate(db,d,row,q.data(),.1f,3.f);
   Check("all trajectory samples contribute, not only final future sample",Near(result.ratio,expected));
   if(weightedDenominator>0&&std::fabs(weightedNumerator/weightedDenominator-expected)>1e-3)weightedOracleDistinct=true;
  }
 }
 Check("all actual databases covered",tested==db.NumDatabases());Check("all actual schemas covered",schemas.size()==db.Header().numSchemas);Check("multiple velocity samples exercised",multiple>0);
 Check("real normalization makes raw oracle distinguishable from weighted magnitudes",weightedOracleDistinct);
 std::printf("NRMV unequal components=%d; %d PASS %d FAIL\n",unequalScale,passed,failed);return failed?1:0;
}
