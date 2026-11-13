// UTPP port report 4.2 #32: minimal montage player (slot DefaultSlot / "Transition", traversal + get-up montages).
// Header-only, no game hooks. Semantics follow UE 5.8 FAnimMontageInstance (Runtime/Engine/Private/Animation/
// AnimMontage.cpp) in the parts motion uses:
//   - position advances by dt * playRate * RateScale; sections play in order, a section with NextSection jumps there,
//     without one the montage ends at the section end (single "Default" section in all 42 motion montages);
//   - blend in over BlendIn.BlendTime with its option; auto blend-out (bEnableAutoBlendOut) starts when the remaining
//     time <= BlendOutTriggerTime (< 0 = BlendOut.BlendTime); Stop(time, option) = Montage_StopWithBlendSettings;
//   - events: BlendOut (auto or stop), Completed (reached the end), Interrupted (stopped before the end),
//     notify state Begin/End and notify Event when the play position crosses them (no double fire, closed on stop);
//   - root motion: delta of the root track (bone Trajectory of the retargeted sequence) between the previous and the
//     new position, expressed in the previous root frame (UE ExtractRootMotionFromRange: inv(T0) * T1), plus the slot
//     weight for the caller (RootMotionFromMontagesOnly weights the montage root motion by its blend weight).
// Blend options: Linear = t; Cubic / HermiteCubic = 3t^2 - 2t^3 (AlphaBlend.cpp 244-247).
#pragma once
#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

namespace utpp::mm::montage {

enum class Blend { Linear, Cubic, HermiteCubic };
inline Blend ParseBlend(const std::string& s) { return s == "Linear" ? Blend::Linear : s == "Cubic" ? Blend::Cubic : Blend::HermiteCubic; }
inline float BlendAlpha(float t, Blend b) { t = std::clamp(t, 0.f, 1.f); return b == Blend::Linear ? t : t * t * (3.f - 2.f * t); }

struct Section { std::string name; float start = 0.f; std::string next; };
struct Segment { std::string anim; float startPos = 0.f, animStart = 0.f, animEnd = 0.f, playRate = 1.f; int looping = 1; };
struct Notify { bool state = false; std::string name, cls; float start = 0.f, duration = 0.f; std::string warpTarget; };
struct RootKey { float t[3] = {0, 0, 0}; float q[4] = {0, 0, 0, 1}; };

struct MontageDef {
    std::string name, slot = "DefaultSlot";
    float length = 0.f, rateScale = 1.f;
    float blendInTime = 0.25f, blendOutTime = 0.25f, blendOutTrigger = -1.f;
    Blend blendIn = Blend::Linear, blendOut = Blend::Linear;
    bool autoBlendOut = true;
    std::vector<Section> sections;
    std::vector<Segment> segments;
    std::vector<Notify> notifies;
    float rootFps = 30.f;
    std::vector<RootKey> root;   // per frame of the (first) segment's sequence
};

inline bool LoadMontages(const std::string& path, std::vector<MontageDef>& out, std::string* err = nullptr) {
    std::ifstream f(path);
    auto fail = [&](const char* m) { if (err) *err = m; return false; };
    if (!f) return fail("cannot open");
    std::string line;
    if (!std::getline(f, line) || line.rfind("utpp-montages 1", 0) != 0) return fail("bad header");
    MontageDef cur; bool in = false; int rootN = 0;
    while (std::getline(f, line)) {
        std::istringstream ss(line); std::string tag; ss >> tag;
        if (tag == "M") {
            cur = MontageDef{}; std::string bi, bo; int ab = 1;
            ss >> cur.name >> cur.length >> cur.rateScale >> cur.blendInTime >> bi >> cur.blendOutTime >> bo >> ab
               >> cur.blendOutTrigger >> cur.slot;
            if (!ss) return fail("bad M row");
            cur.blendIn = ParseBlend(bi); cur.blendOut = ParseBlend(bo); cur.autoBlendOut = ab != 0; in = true;
        } else if (!in) {
            continue;
        } else if (tag == "SEC") {
            Section s; ss >> s.name >> s.start >> s.next; if (s.next == "-") s.next.clear(); cur.sections.push_back(s);
        } else if (tag == "SEG") {
            Segment g; ss >> g.anim >> g.startPos >> g.animStart >> g.animEnd >> g.playRate >> g.looping; cur.segments.push_back(g);
        } else if (tag == "N") {
            Notify n; std::string kind; ss >> kind >> n.name >> n.cls >> n.start >> n.duration >> n.warpTarget;
            n.state = kind == "state"; if (n.warpTarget == "-") n.warpTarget.clear(); cur.notifies.push_back(n);
        } else if (tag == "RFPS") {
            ss >> cur.rootFps >> rootN;
        } else if (tag == "R") {
            RootKey k; ss >> k.t[0] >> k.t[1] >> k.t[2] >> k.q[0] >> k.q[1] >> k.q[2] >> k.q[3]; cur.root.push_back(k);
        } else if (tag == "END") {
            if ((int)cur.root.size() != rootN || cur.sections.empty() || cur.segments.empty() || !(cur.length > 0.f))
                return fail("incomplete montage");
            out.push_back(cur); in = false;
        }
    }
    return !out.empty() ? true : fail("no montages");
}

// Rigid transform helpers (quaternion xyzw).
struct Xf { float t[3] = {0, 0, 0}; float q[4] = {0, 0, 0, 1}; };
inline void QMul(const float* a, const float* b, float* o) {
    const float x = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
    const float y = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
    const float z = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
    const float w = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
    o[0] = x; o[1] = y; o[2] = z; o[3] = w;
}
inline void QRot(const float* q, const float* v, float* o) {
    const float p[4] = {v[0], v[1], v[2], 0.f}, qi[4] = {-q[0], -q[1], -q[2], q[3]};
    float t[4], r[4]; QMul(q, p, t); QMul(t, qi, r); o[0] = r[0]; o[1] = r[1]; o[2] = r[2];
}
inline Xf Compose(const Xf& a, const Xf& b) {   // a then b in a's frame: result = a * b
    Xf o; float rt[3]; QRot(a.q, b.t, rt);
    for (int i = 0; i < 3; ++i) o.t[i] = a.t[i] + rt[i];
    QMul(a.q, b.q, o.q); return o;
}
inline Xf Inverse(const Xf& a) {
    Xf o; o.q[0] = -a.q[0]; o.q[1] = -a.q[1]; o.q[2] = -a.q[2]; o.q[3] = a.q[3];
    const float nt[3] = {-a.t[0], -a.t[1], -a.t[2]}; QRot(o.q, nt, o.t); return o;
}

inline Xf SampleRoot(const MontageDef& d, float animTime) {
    Xf o;
    if (d.root.empty()) return o;
    const float f = std::clamp(animTime * d.rootFps, 0.f, float(d.root.size() - 1));
    const size_t i0 = size_t(f), i1 = std::min(i0 + 1, d.root.size() - 1);
    const float a = f - float(i0);
    const RootKey& k0 = d.root[i0]; const RootKey& k1 = d.root[i1];
    for (int i = 0; i < 3; ++i) o.t[i] = k0.t[i] + (k1.t[i] - k0.t[i]) * a;
    float dot = 0.f; for (int i = 0; i < 4; ++i) dot += k0.q[i] * k1.q[i];
    const float s = dot < 0.f ? -1.f : 1.f; float n = 0.f;
    for (int i = 0; i < 4; ++i) { o.q[i] = k0.q[i] + (s * k1.q[i] - k0.q[i]) * a; n += o.q[i] * o.q[i]; }
    n = std::sqrt(n); for (float& c : o.q) c /= n;
    return o;
}

enum class Event { NotifyBegin, NotifyEnd, NotifyFire, BlendOut, Completed, Interrupted };

class MontagePlayer {
public:
    using Listener = std::function<void(Event, const Notify*)>;
    void SetListener(Listener l) { listener_ = std::move(l); }

    void Play(const MontageDef* def, float playRate = 1.f, float startTime = 0.f) {
        def_ = def; rate_ = playRate; pos_ = std::clamp(startTime, 0.f, def->length);
        playing_ = true; blendingOut_ = false; stopped_ = false; blendTime_ = 0.f;
        completedSent_ = false; interrupted_ = false; notifiesOn_ = true;
        weight_ = def->blendInTime > 0.f ? 0.f : 1.f;
        open_.assign(def->notifies.size(), false);
        // a play start inside a notify state window opens it (UE fires NotifyBegin on the first tick inside)
        for (size_t i = 0; i < def->notifies.size(); ++i) {
            const Notify& n = def->notifies[i];
            if (n.state && pos_ >= n.start && pos_ < n.start + n.duration) { open_[i] = true; Emit(Event::NotifyBegin, &n); }
        }
    }
    // Montage_StopWithBlendSettings
    void Stop(float blendOutTime, Blend option) {
        if (!playing_ || blendingOut_) return;
        BeginBlendOut(blendOutTime, option, true);
    }
    bool Playing() const { return playing_; }
    bool BlendingOut() const { return blendingOut_; }
    float Position() const { return pos_; }
    float Weight() const { return weight_; }
    const Xf& RootDelta() const { return rootDelta_; }   // this tick, in the previous root frame

    // Advance one tick. Returns false once the montage is fully blended out / finished.
    bool Update(float dt) {
        rootDelta_ = Xf{};
        if (!playing_ || !def_) return false;
        const float prev = pos_;
        if (!stopped_) {
            float next = pos_ + dt * rate_ * def_->rateScale;
            const float end = SectionEnd(prev);
            if (next >= end) {
                const Section* s = NextSection(prev);
                if (s) next = s->start + (next - end);
                else next = end;
            }
            Advance(prev, std::min(next, def_->length));
        }
        blendTime_ += dt;
        if (!blendingOut_) {
            weight_ = def_->blendInTime > 0.f ? BlendAlpha(blendTime_ / def_->blendInTime, def_->blendIn) : 1.f;
            const float trigger = def_->blendOutTrigger < 0.f ? def_->blendOutTime : def_->blendOutTrigger;
            if (def_->autoBlendOut && !stopped_ && def_->length - pos_ <= trigger + 1e-6f)
                BeginBlendOut(def_->blendOutTime, def_->blendOut, false);
        }
        if (blendingOut_) {
            const float a = outTime_ > 0.f ? BlendAlpha((blendTime_ - outStart_) / outTime_, outOpt_) : 1.f;
            weight_ = outFrom_ * (1.f - a);
        }
        if (!stopped_ && pos_ >= def_->length - 1e-6f && !NextSection(pos_)) {
            CloseAll();
            if (!blendingOut_) BeginBlendOut(0.f, Blend::Linear, false);
            if (!completedSent_ && !interrupted_) { completedSent_ = true; Emit(Event::Completed, nullptr); }
            stopped_ = true;
        }
        if (blendingOut_ && weight_ <= 0.f) {
            weight_ = 0.f; playing_ = false;
            CloseAll();   // the instance terminates: open notify states end (also when the blend-out ends a tick early)
            if (!completedSent_ && !interrupted_) { completedSent_ = true; Emit(Event::Completed, nullptr); }
        }
        return playing_;
    }

private:
    float SectionEnd(float p) const {
        float end = def_->length;
        for (const auto& s : def_->sections) if (s.start > p + 1e-6f) end = std::min(end, s.start);
        return end;
    }
    const Section* NextSection(float p) const {
        const Section* cur = nullptr;
        for (const auto& s : def_->sections) if (s.start <= p + 1e-6f && (!cur || s.start > cur->start)) cur = &s;
        if (!cur || cur->next.empty()) return nullptr;
        for (const auto& s : def_->sections) if (s.name == cur->next) return &s;
        return nullptr;
    }
    float AnimTime(float p) const {
        for (const auto& g : def_->segments) {
            const float len = (g.animEnd - g.animStart) / std::max(g.playRate, 1e-6f);
            if (p >= g.startPos - 1e-6f && p <= g.startPos + len + 1e-6f) return g.animStart + (p - g.startPos) * g.playRate;
        }
        return p;
    }
    void Advance(float from, float to) {
        // root motion (UE ExtractRootMotionFromRange): inv(T(from)) * T(to); section jumps are handled as two ranges
        if (to >= from) {
            rootDelta_ = Compose(Inverse(SampleRoot(*def_, AnimTime(from))), SampleRoot(*def_, AnimTime(to)));
            Cross(from, to);
        } else {
            const float end = SectionEnd(from);
            rootDelta_ = Compose(Inverse(SampleRoot(*def_, AnimTime(from))), SampleRoot(*def_, AnimTime(end)));
            Cross(from, end);
            const Section* s = NextSection(from);
            const float st = s ? s->start : to;
            rootDelta_ = Compose(rootDelta_, Compose(Inverse(SampleRoot(*def_, AnimTime(st))), SampleRoot(*def_, AnimTime(to))));
            Cross(st, to);
        }
        pos_ = to;
    }
    void Cross(float from, float to) {
        if (!notifiesOn_) return;
        for (size_t i = 0; i < def_->notifies.size(); ++i) {
            const Notify& n = def_->notifies[i];
            if (!n.state) { if (n.start > from && n.start <= to) Emit(Event::NotifyFire, &n); continue; }
            const float e = n.start + n.duration;
            if (!open_[i] && n.start >= from && n.start < to && e > from) { open_[i] = true; Emit(Event::NotifyBegin, &n); }
            if (open_[i] && e > from && e <= to) { open_[i] = false; Emit(Event::NotifyEnd, &n); }
        }
    }
    void CloseAll() {
        for (size_t i = 0; i < open_.size(); ++i) if (open_[i]) { open_[i] = false; Emit(Event::NotifyEnd, &def_->notifies[i]); }
    }
    void BeginBlendOut(float time, Blend opt, bool interrupted) {
        blendingOut_ = true; outTime_ = time; outOpt_ = opt; outStart_ = blendTime_; outFrom_ = weight_;
        Emit(Event::BlendOut, nullptr);
        // UE keeps ticking the instance while it blends out (position + root motion); notify states are closed
        if (interrupted) { interrupted_ = true; CloseAll(); notifiesOn_ = false; Emit(Event::Interrupted, nullptr); }
    }
    void Emit(Event e, const Notify* n) { if (listener_) listener_(e, n); }

    const MontageDef* def_ = nullptr;
    Listener listener_;
    float rate_ = 1.f, pos_ = 0.f, weight_ = 0.f, blendTime_ = 0.f;
    float outTime_ = 0.f, outStart_ = 0.f, outFrom_ = 1.f;
    Blend outOpt_ = Blend::Linear;
    bool playing_ = false, blendingOut_ = false, stopped_ = false, completedSent_ = false, interrupted_ = false;
    bool notifiesOn_ = true;
    std::vector<bool> open_;
    Xf rootDelta_;
};

}  // namespace utpp::mm::montage
