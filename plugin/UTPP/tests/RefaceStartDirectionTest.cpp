// Regression from the 2026-10-18 23:44:20 recording.
// Uses the real recorded-feed driver replay. Actor yaw is recorded, not a
// counterfactual Lua/controller simulation. Fixture: woman DB SHA256 392C5755...
#define UTPP_REPLAY_MAIN ReplayRefaceRecordedFeed
#include "../../../tools/analysis/replay_locomotion_stages.cpp"
#undef UTPP_REPLAY_MAIN

namespace {
struct RefaceRow { float time; int clip; float clipTime; float actorYaw, offsetYaw; };
std::vector<std::string> CsvCells(const std::string& line) {
    std::vector<std::string> cells;
    std::istringstream stream(line);
    std::string value;
    while (std::getline(stream, value, ',')) cells.push_back(value);
    return cells;
}
// These IDs belong to the exact recorded woman fixture, not a general asset rule.
int RefaceDirection(int clip) {
    return clip == 397 ? 1 : clip == 399 ? -1 : 0;
}
}

int main(int argc, char** argv) {
    if (argc != 5 && argc != 6) {
        std::puts("usage RefaceStartDirectionTest <driver-data> <basedb> <recorded-feed> <output-csv> [woman-db]");
        return 2;
    }
    const int replayResult = ReplayRefaceRecordedFeed(argc, argv);
    if (replayResult != 0) return replayResult;
    std::ifstream input(argv[4]);
    std::string line;
    std::getline(input, line);
    const auto header = CsvCells(line);
    size_t actorColumn = header.size(), offsetColumn = header.size();
    for (size_t i = 0; i < header.size(); ++i) {
        if (header[i] == "actorYaw") actorColumn = i;
        if (header[i] == "offYaw") offsetColumn = i;
    }
    if (actorColumn == header.size() || offsetColumn == header.size()) return 7;
    std::vector<RefaceRow> rows;
    while (std::getline(input, line)) {
        const auto cells = CsvCells(line);
        if (cells.size() <= std::max(actorColumn, offsetColumn)) return 7;
        rows.push_back({std::stof(cells[0]), std::stoi(cells[3]), std::stof(cells[4]),
                        std::stof(cells[actorColumn]), std::stof(cells[offsetColumn])});
    }
    int failures = 0;
    // First, third, and final idle->backward starts; second is the opposite
    // direction control. All have a constant target and uninterrupted input.
    for (const float start : {61.156014f, 62.755292f, 65.055261f, 73.855644f}) {
        int initial = 0, samples = 0, flips = 0;
        const int expected = start == 62.755292f ? -1 : 1;
        bool haveVisual = false;
        float lastVisual = 0.0f, visualProgress = 0.0f, worstBackwardStep = 0.0f;
        for (const auto& row : rows) {
            if (row.time < start - 0.001f || row.time > start + 0.75f) continue;
            const float visual = row.actorYaw + row.offsetYaw;
            if (haveVisual) {
                const float signedStep = std::remainder(visual - lastVisual, 360.0f) * expected;
                visualProgress += signedStep;
                worstBackwardStep = std::min(worstBackwardStep, signedStep);
            }
            haveVisual = true;
            lastVisual = visual;
            const int direction = RefaceDirection(row.clip);
            if (!direction) continue;
            if (!initial) initial = direction;
            ++samples;
            if (direction != initial) ++flips;
        }
        const bool passed = samples > 15 && flips == 0 && initial == expected &&
                            visualProgress > 90.0f && worstBackwardStep > -5.0f;
        failures += !passed;
        std::printf("%s recorded start %.6f: expected=%d first=%d samples=%d flips=%d visualProgress=%.2f worstBackwardStep=%.2f\n",
                    passed ? "PASS" : "FAIL", start, expected, initial, samples, flips,
                    visualProgress, worstBackwardStep);
    }
    std::printf("Recorded actor yaw retained; Lua/controller behavior is outside this test. failures=%d\n", failures);
    return failures ? 1 : 0;
}
