#include "../src/mm/MappingByteReceipt.hpp"
#include <array>
#include <cstdio>
int main()
{
    namespace r=utpp::mm::mappingreceipt;
    int checks=0,failures=0;
    auto check=[&](bool ok){++checks;if(!ok)++failures;};
    std::array<std::byte,96> source{},destination{};
    std::array<uint32_t,4> pairs{0,1,1,0};
    auto result=r::Compare(source,destination,pairs);
    check(result.valid&&result.compared==2&&result.equal==2&&!result.overlap);
    destination[47]=std::byte{1}; // Includes the W/tag lanes, not just ten TRS floats.
    result=r::Compare(source,destination,pairs);
    check(result.valid&&result.equal==1&&result.compared==2);
    check(source[47]==std::byte{0}&&destination[47]==std::byte{1});
    pairs[3]=2;
    result=r::Compare(source,destination,pairs);
    check(!result.valid&&result.compared==0);
    pairs[3]=0;pairs[2]=2;
    check(!r::Compare(source,destination,pairs).valid);
    check(!r::Compare(source,destination,std::span(pairs).first(3)).valid);
    check(!r::Compare(source,destination,{}).valid);
    check(!r::Compare(std::span(source).first(95),destination,pairs).valid);
    result=r::Compare(source,source,std::span(pairs).first(2));
    check(!result.valid&&result.overlap);
    result=r::Compare(std::span(source).first(48),std::span(source).last(48),std::array<uint32_t,2>{0,0});
    check(result.valid&&!result.overlap); // Adjacent buffers are disjoint.
    result=r::Compare(source,std::span(source).last(48),std::array<uint32_t,2>{0,0});
    check(!result.valid&&result.overlap);
    std::array<std::byte,65*48> manySource{},manyDestination{};
    std::array<uint32_t,130> manyPairs{};
    for(uint32_t i=0;i<65;++i) manyPairs[i*2]=manyPairs[i*2+1]=i;
    result=r::Compare(manySource,manyDestination,manyPairs);
    check(result.valid&&result.equalPairs[0]==~uint64_t{0}&&result.equalPairs[1]==1);
    manyDestination[64*48+47]=std::byte{1};
    result=r::Compare(manySource,manyDestination,manyPairs);
    check(result.valid&&result.equal==64&&result.equalPairs[0]==~uint64_t{0}&&result.equalPairs[1]==0);
    std::printf("MappingByteReceipt: %d checks, %d failures\n",checks,failures);
    return failures?1:0;
}
