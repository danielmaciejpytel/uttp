#include "../src/mm/MotionRootFrame.hpp"
#include <cstdio>
#include <limits>
using namespace utpp::mm;
static int pass = 0, fail = 0;
static void C(const char *n, bool v)
{
    std::printf("%s %s\n", v ? "PASS" : "FAIL", n);
    v ? ++pass : ++fail;
}
static bool Near(float a, float b, float e = 2e-5f)
{
    return std::isfinite(a) && std::fabs(a - b) < e;
}
int main()
{
    using namespace rootframe;
    Processor p;
    p.Reset(.3f);
    Inputs in;
    in.dt = .016f;
    in.actorYaw = .5f;
    in.capsuleYawDelta = .2f;
    in.capsuleDeltaCurrent = {.05f, .03f, 0};
    in.animationDeltaPreviousVisual = {.02f, .01f, 0};
    in.animationYawDelta = .01f;
    in.movingGround = true;
    Proposal first;
    C("first prepare succeeds without mutating processor",
      p.Prepare(in, first) && p.Sequence() == 0 && Near(p.PreviousVisualWorldYaw(), .3f));
    C("first feed discards motion and reseeds actual actor", first.Reseeded() == Reseed::First &&
                                                                 first.Frame().x == 0 && first.Frame().yaw == 0 &&
                                                                 Near(first.VisualWorldYaw(), .5f));
    C("commit publishes first feed", p.Commit(first) && p.Sequence() == 1);
    C("proposal cannot commit twice", !p.Commit(first));
    Proposal abandoned;
    p.Prepare(in, abandoned);
    const auto before = p.Frame();
    const float oldVisual = p.PreviousVisualWorldYaw();
    const auto oldSequence = p.Sequence();
    C("abandoned prepare does not advance root",
      Near(p.Frame().x, before.x) && Near(p.PreviousVisualWorldYaw(), oldVisual) && p.Sequence() == oldSequence);
    Inputs invalid = in;
    invalid.animationYawDelta = std::numeric_limits<float>::infinity();
    Proposal sentinel = abandoned;
    C("invalid prepare preserves candidate and live state",
      !p.Prepare(invalid, sentinel) && sentinel.Sequence() == abandoned.Sequence() &&
          sentinel.VisualWorldYaw() == abandoned.VisualWorldYaw() && p.Sequence() == oldSequence);
    Processor other;
    other.Reset(.5f);
    C("cross-owner candidate rejected", !other.Commit(abandoned));
    Proposal a, b;
    p.Prepare(in, a);
    p.Prepare(in, b);
    C("first of competing proposals commits", p.Commit(a));
    C("stale parallel proposal rejected", !p.Commit(b));
    p.Prepare(in, b);
    p.Reset(.8f);
    C("reset invalidates pending epoch", !p.Commit(b));
    motionRoot::OffsetRoot oracle;
    p.Reset(.8f);
    in.actorYaw = .8f;
    in.capsuleYawDelta = 0;
    p.Prepare(in, first);
    p.Commit(first);
    float actor = .8f;
    for (int n = 0; n < 80; ++n)
    {
        in.dt = n % 2 ? .017f : .013f;
        in.capsuleYawDelta = .03f;
        actor += in.capsuleYawDelta;
        in.actorYaw = actor;
        in.animationYawDelta = .01f;
        in.movingGround = n < 50;
        in.montage = n >= 70;
        oracle.Update(in.dt, in.capsuleDeltaCurrent.x, in.capsuleDeltaCurrent.y, in.capsuleYawDelta,
                      in.animationDeltaPreviousVisual.x, in.animationDeltaPreviousVisual.y, in.animationYawDelta,
                      in.movingGround, in.montage);
        Proposal next;
        bool ok = p.Prepare(in, next);
        C("candidate exactly matches standalone native-root kernel", ok && Near(next.Frame().x, oracle.X()) &&
                                                                         Near(next.Frame().y, oracle.Y()) &&
                                                                         Near(next.Frame().yaw, oracle.Yaw()));
        C("candidate visual heading same-feed actor plus offset",
          Near(std::sin(next.VisualWorldYaw()), std::sin(actor + oracle.Yaw())));
        C("oracle candidate commits", p.Commit(next));
    }
    const auto prior = p.Frame();
    const float priorVisual = p.PreviousVisualWorldYaw();
    in.dt = 0;
    in.actorYaw = 1.9f;
    Proposal zero;
    p.Prepare(in, zero);
    p.Commit(zero);
    C("dt0 retains published physical state", p.Frame().x == prior.x && p.Frame().y == prior.y &&
                                                  p.Frame().yaw == prior.yaw &&
                                                  p.PreviousVisualWorldYaw() == priorVisual);
    in.dt = .251f;
    Proposal gap;
    const auto oldEpoch = p.Epoch();
    C("original gap prepares reseed instead of integrating", p.Prepare(in, gap) && gap.Reseeded() == Reseed::Gap &&
                                                                 gap.Frame().x == 0 && gap.Frame().yaw == 0 &&
                                                                 gap.Epoch() != oldEpoch);
    C("abandoned gap leaves current epoch unchanged", p.Epoch() == oldEpoch);
    p.Commit(gap);
    C("gap commit publishes reseeded heading", Near(p.PreviousVisualWorldYaw(), in.actorYaw));
    in.dt = .016f;
    in.teleport = true;
    Proposal tele;
    C("explicit teleport prepares reset", p.Prepare(in, tele) && tele.Reseeded() == Reseed::Teleport);
    p.Commit(tele);
    in.teleport = false;
    in.capsuleDeltaCurrent = {.501f, 0, 0};
    C("capsule displacement above half metre resets", p.Prepare(in, tele) && tele.Reseeded() == Reseed::Teleport);
    p.Commit(tele);
    in.capsuleDeltaCurrent = {.01f, 0, 0};
    in.staticTailWeight = .2f;
    in.rootReliable = true;
    Proposal approx;
    p.Prepare(in, approx);
    C("static tail marks root approximate", !approx.RootReliable() && Near(approx.StaticTailWeight(), .2f));
    in.staticTailWeight = -.1f;
    C("invalid tail weight rejected", !p.Prepare(in, approx));
    C("invalid reset preserves epoch", !p.Reset(std::numeric_limits<float>::quiet_NaN()));
    Processor x, y;
    x.Reset(0);
    y.Reset(1.1f);
    in = {};
    in.dt = .016f;
    x.Prepare(in, a);
    x.Commit(a);
    in.actorYaw = 1.1f;
    y.Prepare(in, b);
    y.Commit(b);
    for (int n = 0; n < 40; ++n)
    {
        in.dt = .015f;
        in.movingGround = true;
        in.capsuleYawDelta = .04f;
        in.animationYawDelta = .012f;
        in.capsuleDeltaCurrent = {.02f, .01f, 0};
        in.animationDeltaPreviousVisual = {.01f, .02f, 0};
        in.actorYaw = .04f * (n + 1);
        x.Prepare(in, a);
        in.actorYaw += 1.1f;
        y.Prepare(in, b);
        C("global yaw covariance preserves local offsets",
          Near(a.Frame().x, b.Frame().x) && Near(a.Frame().y, b.Frame().y) && Near(a.Frame().yaw, b.Frame().yaw));
        C("global yaw covariance shifts only visual world heading",
          Near(std::sin(b.VisualWorldYaw() - a.VisualWorldYaw()), std::sin(1.1f)));
        x.Commit(a);
        y.Commit(b);
    }
    std::printf("%d PASS %d FAIL\n", pass, fail);
    return fail ? 1 : 0;
}
