// UTPP Nue reload graph patcher (offline only; never writes outside the paths given on the command line).
//
// Faithful C# port of Source\tools\analysis\patch_nue_reload_graph_twohand.ps1 (-IncludeNormalReload) driven by
// verify_cr2w_graph_schema_roundtrip.ps1. The PowerShell 7 host used for the 2.31 build is not available on this
// machine, and WolvenKit.CLI 9.0.1 targets net10.0, so the same algorithm runs here as a small net10 tool.
//
// The UTPP delta is NOT an event rename: it wraps the native EMPTY (empty_reload, _01, _02) and NORMAL
// (reload, _01, _02) animAnimNode_SkSpeedAnim leaves of the game's own graph in three private bool gates
// (features UTPPNueReload / UTPPNueReloadAim / UTPPNueReload2H). The OFF branch is the very same native leaf object,
// so the game's own (2.5) states, events (RightReload, EndRightReload, ...) and features stay untouched.
//
// usage: NueGraphPatch <body|magazine> <source.animgraph> <output.animgraph> <report.json> [--expect-sha <hex>]
//        NueGraphPatch roundtrip <source.animgraph> <output.animgraph> <report.json>   (no patch, pure re-write)
//        NueGraphPatch compare <a.animgraph> <b.animgraph> <report.json>              (deep object compare only)
using System.Reflection;
using System.Runtime.Loader;
using System.Security.Cryptography;
using System.Text.Json;
using WolvenKit.RED4.Types;

static class Program
{
    static string WkDir;

    static int Main(string[] args)
    {
        WkDir = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, Environment.GetEnvironmentVariable("UTPP_WK_DIR") ?? ""));
        if (!File.Exists(Path.Combine(WkDir, "WolvenKit.RED4.dll"))) { Console.Error.WriteLine("Set UTPP_WK_DIR to Tools\\WolvenKit.CLI"); return 2; }
        WkDir = WkDir.TrimEnd('\\') + "\\";
        AppContext.SetData("APP_CONTEXT_BASE_DIRECTORY", WkDir);
        AssemblyLoadContext.Default.Resolving += (ctx, name) =>
        {
            var p = Path.Combine(WkDir, name.Name + ".dll");
            return File.Exists(p) ? ctx.LoadFromAssemblyPath(p) : null;
        };
        try { return Run(args); }
        catch (Exception e) { Console.Error.WriteLine("FAIL: " + e); return 1; }
    }

    static int Run(string[] args)
    {
        if (args.Length < 4) { Console.Error.WriteLine("usage: <body|magazine|roundtrip> <src> <out> <report> [--expect-sha HEX]"); return 2; }
        string mode = args[0], src = Path.GetFullPath(args[1]), outp = Path.GetFullPath(args[2]), report = Path.GetFullPath(args[3]);
        string expect = null;
        for (int i = 4; i + 1 < args.Length; i++) if (args[i] == "--expect-sha") expect = args[i + 1];
        if ((mode != "compare" && File.Exists(outp)) || File.Exists(report)) throw new Exception("Output or report exists");
        if (mode == "body" && Path.GetFileName(src) != "player_ranged.animgraph" && Path.GetFileName(src) != "player_dualwield.animgraph" && !src.Contains("body")) throw new Exception("Body mode requires player graph");
        if (mode == "magazine" && Path.GetFileName(src) != "weapon.animgraph" && !src.Contains("weapon")) throw new Exception("Magazine mode requires weapon graph");

        // Same host bootstrap as the PowerShell verifier (initialises WolvenKit services / hash caches).
        var cli = Assembly.LoadFrom(Path.Combine(WkDir, "WolvenKit.CLI.dll"));
        var hostBuilder = cli.GetType("WolvenKit.CLI.GenericHost").GetMethod("CreateHostBuilder").Invoke(null, new object[] { Array.Empty<string>() });
        var host = (IDisposable)hostBuilder.GetType().GetMethod("Build").Invoke(hostBuilder, null);
        try
        {
            var red4 = typeof(RedBaseClass).Assembly;
            var unknowns = new HashSet<string>();
            ParsingErrorEventHandler handler = info => { if (info is UnknownTypeEventArgs u) unknowns.Add(u.RedName); return false; };

            if (mode == "compare")
            {
                // compare <a> <b> <report>: deep object comparison of two existing CR2W files (no file writes but report).
                var fa = ReadCr2w(red4, src, handler, out var ca);
                var fb = ReadCr2w(red4, outp, handler, out var cb);
                if (unknowns.Count > 0 || ca != "NoError" || cb != "NoError") throw new Exception("Read failed: " + ca + "/" + cb + " " + string.Join(",", unknowns));
                var c = Compare.Deep(fa.GetType().GetProperty("RootChunk").GetValue(fa), fb.GetType().GetProperty("RootChunk").GetValue(fb));
                var cr = new Dictionary<string, object> { ["mode"] = mode, ["a"] = src, ["aSHA256"] = Sha(src), ["b"] = outp, ["bSHA256"] = Sha(outp),
                    ["checkedValues"] = c.Checked, ["reachableClasses"] = c.Classes, ["failures"] = c.Failures };
                File.WriteAllText(report, JsonSerializer.Serialize(cr, new JsonSerializerOptions { WriteIndented = true }));
                Console.WriteLine(JsonSerializer.Serialize(new { c.Checked, c.Classes, failures = c.Failures.Count }));
                return c.Failures.Count == 0 ? 0 : 1;
            }

            object file = ReadCr2w(red4, src, handler, out var code1);
            Console.WriteLine("READ_RESULT=" + code1);
            Console.WriteLine("UNKNOWNS=" + string.Join(",", unknowns));
            if (unknowns.Count > 0) throw new Exception("Unhandled source type");
            if (code1 != "NoError") throw new Exception("Read failed");

            var root = (RedBaseClass)file.GetType().GetProperty("RootChunk").GetValue(file);
            Dictionary<string, object> patch = null;
            if (mode == "body") patch = Patcher.Apply(root, magazine: false);
            else if (mode == "magazine") patch = Patcher.Apply(root, magazine: true);
            else if (mode != "roundtrip") throw new Exception("unknown mode " + mode);

            Directory.CreateDirectory(Path.GetDirectoryName(outp));
            using (var os = File.Create(outp))
            {
                var writer = System.Activator.CreateInstance(red4.GetType("WolvenKit.RED4.Archive.IO.CR2WWriter"), os);
                writer.GetType().GetMethod("WriteFile").Invoke(writer, new[] { file });
                (writer as IDisposable)?.Dispose();
            }
            Console.WriteLine("OUTPUT_BYTES=" + new FileInfo(outp).Length);

            object second = ReadCr2w(red4, outp, handler, out var code2);
            if (unknowns.Count > 0) throw new Exception("Unhandled roundtrip type");
            if (code2 != "NoError") throw new Exception("Second read failed");
            var root2 = (RedBaseClass)second.GetType().GetProperty("RootChunk").GetValue(second);
            var cmp = Compare.Deep(root, root2);

            string sha = Sha(outp);
            var rep = new Dictionary<string, object>
            {
                ["mode"] = mode,
                ["patch"] = patch,
                ["source"] = src,
                ["sourceSHA256"] = Sha(src),
                ["output"] = outp,
                ["outputSHA256"] = sha,
                ["outputBytes"] = new FileInfo(outp).Length,
                ["checkedValues"] = cmp.Checked,
                ["reachableClasses"] = cmp.Classes,
                ["buffers"] = cmp.Buffers,
                ["failures"] = cmp.Failures,
                ["expectedSHA256"] = expect,
                ["expectedMatch"] = expect == null ? null : (object)string.Equals(expect, sha, StringComparison.OrdinalIgnoreCase),
                ["runtimeVerified"] = false,
            };
            File.WriteAllText(report, JsonSerializer.Serialize(rep, new JsonSerializerOptions { WriteIndented = true }));
            Console.WriteLine(JsonSerializer.Serialize(new { sha, cmp.Checked, cmp.Classes, failures = cmp.Failures.Count, expectedMatch = rep["expectedMatch"] }));
            if (cmp.Failures.Count > 0) throw new Exception("Object graph differs");
            if (expect != null && !(bool)rep["expectedMatch"]) throw new Exception("Expected SHA mismatch");
            return 0;
        }
        finally { host.Dispose(); }
    }

    static object ReadCr2w(Assembly red4, string path, ParsingErrorEventHandler handler, out string code)
    {
        // Load fully into memory so the source file handle is released immediately.
        var ms = new MemoryStream(File.ReadAllBytes(path));
        var reader = System.Activator.CreateInstance(red4.GetType("WolvenKit.RED4.Archive.IO.CR2WReader"), ms);
        reader.GetType().GetEvent("ParsingError").AddEventHandler(reader, handler);
        var read = reader.GetType().GetMethods().First(m => m.Name == "ReadFile" && m.GetParameters().Length == 2);
        var a = new object[] { null, false };
        code = read.Invoke(reader, a).ToString();
        return a[0];
    }

    static string Sha(string p) { using var s = File.OpenRead(p); return Convert.ToHexString(SHA256.HashData(s)); }
}

static class Patcher
{
    static object Get(RedBaseClass c, string n) => c.GetProperty(n);
    static string TypeName(object o) => o?.GetType().Name;

    // Copy-NueRecord: handles are re-created pointing at the SAME target; arrays/classes are deep-copied.
    static object Copy(object v)
    {
        if (v == null) return null;
        if (v is IRedBaseHandle h)
        {
            var c = (IRedBaseHandle)System.Activator.CreateInstance(v.GetType());
            c.SetValue(h.GetValue());
            return c;
        }
        if (v is IRedArray arr)
        {
            var c = (IRedArray)System.Activator.CreateInstance(v.GetType());
            foreach (var e in arr) c.Add((IRedType)Copy(e));
            return c;
        }
        if (v is RedBaseClass rc)
        {
            var c = (RedBaseClass)System.Activator.CreateInstance(v.GetType());
            foreach (var n in rc.GetPropertyNames()) c.SetProperty(n, (IRedType)Copy(rc.GetProperty(n)));
            return c;
        }
        return v;
    }

    static void Link(object link, RedBaseClass target) => ((IRedBaseHandle)((RedBaseClass)link).GetProperty("node")).SetValue(target);

    public static Dictionary<string, object> Apply(RedBaseClass root, bool magazine)
    {
        string clipPrefix = magazine ? "utpp_nue_empty_reload_mag_" : "utpp_nue_empty_reload_";
        var nodes = new List<RedBaseClass>();
        var handles = new List<IRedBaseHandle>();
        var seen = new HashSet<object>(ReferenceEqualityComparer.Instance);
        var pending = new Stack<object>(); pending.Push(root);
        while (pending.Count > 0)
        {
            var v = pending.Pop(); if (v == null) continue;
            if (v is IRedBaseHandle h) { handles.Add(h); pending.Push(h.GetValue()); }
            else if (v is RedBaseClass c)
            {
                if (!seen.Add(c)) continue; nodes.Add(c);
                foreach (var n in c.GetPropertyNames()) pending.Push(c.GetProperty(n));
            }
            else if (v is IRedArray a) { foreach (var e in a) pending.Push(e); }
        }

        RedBaseClass template = null, converter = null, boolInput = null;
        foreach (var node in nodes)
        {
            if (TypeName(node) != "animAnimNode_Switch") continue;
            if (((IRedArray)Get(node, "inputNodes")).Count != 2) continue;
            var weight = ((IRedBaseHandle)((RedBaseClass)Get(node, "weightNode")).GetProperty("node")).GetValue();
            if (weight == null || TypeName(weight) != "animAnimNode_BoolToFloatConverter") continue;
            var input = ((IRedBaseHandle)((RedBaseClass)weight.GetProperty("inputNode")).GetProperty("node")).GetValue();
            if (input != null && TypeName(input) == "animAnimNode_BoolInput") { template = node; converter = weight; boolInput = input; break; }
        }
        if (template == null) throw new Exception("Native bool switch template missing");

        string Anim(RedBaseClass n) => Get(n, "animation").ToString();
        var targets = nodes.Where(n => TypeName(n) == "animAnimNode_SkSpeedAnim" && new[] { "empty_reload", "empty_reload_01", "empty_reload_02" }.Contains(Anim(n))).ToList();
        if (targets.Count != 6) throw new Exception("Unexpected EMPTY leaf count: " + targets.Count);
        var normal = nodes.Where(n => TypeName(n) == "animAnimNode_SkSpeedAnim" && new[] { "reload", "reload_01", "reload_02" }.Contains(Anim(n))).ToList();
        if (normal.Count != 3) throw new Exception("Unexpected NORMAL leaf count: " + normal.Count);
        foreach (var clip in new[] { "reload", "reload_01", "reload_02" })
            if (normal.Count(n => Anim(n) == clip) != 1) throw new Exception("NORMAL leaf missing or duplicated: " + clip);
        targets.AddRange(normal);

        var newNodes = new List<RedBaseClass>();
        RedBaseClass Gate(string group, RedBaseClass falseNode, RedBaseClass trueNode)
        {
            var b = (RedBaseClass)Copy(boolInput); b.SetProperty("group", (CName)group); b.SetProperty("name", (CName)"IsActive");
            var c = (RedBaseClass)Copy(converter); Link(c.GetProperty("inputNode"), b);
            var g = (RedBaseClass)Copy(template);
            var ins = (IRedArray)g.GetProperty("inputNodes");
            Link(ins[0], falseNode); Link(ins[1], trueNode);
            Link(g.GetProperty("weightNode"), c);
            newNodes.Add(b); newNodes.Add(c); newNodes.Add(g); return g;
        }

        var featureList = (IRedArray)Get(root, "animFeatures");
        var initList = (IRedArray)Get(root, "nodesToInit");
        int beforeInit = initList.Count, beforeFeatures = featureList.Count;
        bool backendPresent = false;
        if (root.GetPropertyNames().Contains("backendData"))
        {
            var bh = Get(root, "backendData") as IRedBaseHandle;
            var bd = bh?.GetValue();
            if (bd != null)
            {
                backendPresent = true;
                if (!bd.GetPropertyNames().Contains("graph")) throw new Exception("Unknown backend shape");
                var bg = ((IRedBaseHandle)bd.GetProperty("graph")).GetValue();
                if (!ReferenceEquals(bg, root)) throw new Exception("Unexpected backend/runtime graph alias");
            }
        }
        var featureTemplate = featureList.Cast<RedBaseClass>().Where(f => f.GetProperty("className").ToString() == "gameAnimFeature_TPPRepresentation").ToList();
        if (featureTemplate.Count == 0 && magazine) featureTemplate = new List<RedBaseClass> { (RedBaseClass)featureList[0] };
        if (featureTemplate.Count != 1) throw new Exception("Feature template missing or ambiguous");
        foreach (var group in new[] { "UTPPNueReload", "UTPPNueReloadAim", "UTPPNueReload2H" })
        {
            if (featureList.Cast<RedBaseClass>().Any(f => f.GetProperty("name").ToString() == group)) throw new Exception("Gate already exists");
            var f = (RedBaseClass)Copy(featureTemplate[0]);
            f.SetProperty("className", (CName)"gameAnimFeature_TPPRepresentation"); f.SetProperty("name", (CName)group);
            featureList.Add(f);
        }

        var checks = new List<Dictionary<string, object>>();
        var initTemplate = initList[0];
        foreach (var native in targets)
        {
            var prefix = clipPrefix;
            if (!Anim(native).StartsWith("empty_")) prefix = magazine ? "utpp_nue_reload_mag_" : "utpp_nue_reload_";
            RedBaseClass Clip(string suffix) { var x = (RedBaseClass)Copy(native); x.SetProperty("animation", (CName)(prefix + suffix)); return x; }
            var ready = Clip("ready"); var aim = Clip("aim"); var ready2H = Clip("2h_ready"); var aim2H = Clip("2h_aim");
            newNodes.Add(ready); newNodes.Add(aim); newNodes.Add(ready2H); newNodes.Add(aim2H);
            var readyChoice = Gate("UTPPNueReload2H", ready, ready2H);
            var aimChoice = Gate("UTPPNueReload2H", aim, aim2H);
            var inner = Gate("UTPPNueReloadAim", readyChoice, aimChoice);
            var outer = Gate("UTPPNueReload", native, inner);
            int changed = 0;
            foreach (var h in handles) if (ReferenceEquals(h.GetValue(), native)) { h.SetValue(outer); changed++; }
            if (changed == 0) throw new Exception("Unreferenced native leaf");
            var falseBranch = ((IRedBaseHandle)((RedBaseClass)((IRedArray)outer.GetProperty("inputNodes"))[0]).GetProperty("node")).GetValue();
            if (!ReferenceEquals(falseBranch, native)) throw new Exception("Native fallback changed");
            foreach (var n in new[] { native }.Concat(newNodes.Where(x => !ReferenceEquals(x, outer))))
            {
                var hh = (IRedBaseHandle)Copy(initTemplate); hh.SetValue(n); initList.Add(hh);
            }
            checks.Add(new Dictionary<string, object>
            {
                ["nativeAnimation"] = Anim(native), ["clipPrefix"] = prefix, ["incomingHandles"] = changed, ["nativeFalseBranchSameObject"] = true,
                ["oneHandReady"] = prefix + "ready", ["oneHandAim"] = prefix + "aim", ["twoHandReady"] = prefix + "2h_ready", ["twoHandAim"] = prefix + "2h_aim",
            });
            newNodes.Clear();
        }
        int expectedAdded = 16 * targets.Count;
        if (initList.Count - beforeInit != expectedAdded || featureList.Count - beforeFeatures != 3) throw new Exception("Unexpected graph additions");
        return new Dictionary<string, object>
        {
            ["editorBackendPresent"] = backendPresent, ["nativeInitCount"] = beforeInit, ["finalInitCount"] = initList.Count,
            ["clipPrefix"] = clipPrefix, ["includeNormalReload"] = true, ["wrappedLeaves"] = checks, ["addedInitNodes"] = expectedAdded,
            ["privateFeatures"] = new[] { "UTPPNueReload", "UTPPNueReloadAim", "UTPPNueReload2H" }, ["deployable"] = false, ["runtimeVerified"] = false,
        };
    }
}

class Compare
{
    public int Checked, Classes, Buffers;
    public List<string> Failures = new();

    public static Compare Deep(object a, object b)
    {
        var r = new Compare();
        var fwd = new Dictionary<object, object>(ReferenceEqualityComparer.Instance);
        var rev = new Dictionary<object, object>(ReferenceEqualityComparer.Instance);
        var st = new Stack<(object, object, string)>(); st.Push((a, b, "root"));
        while (st.Count > 0)
        {
            var (l, rr, path) = st.Pop(); r.Checked++;
            if (l == null || rr == null) { if (l != null || rr != null) r.Failures.Add(path + ": null"); continue; }
            if (l.GetType() != rr.GetType()) { r.Failures.Add(path + ": type"); continue; }
            if (l is RedBaseClass lc)
            {
                if (fwd.TryGetValue(l, out var m)) { if (!ReferenceEquals(m, rr)) r.Failures.Add(path + ": alias"); continue; }
                if (rev.ContainsKey(rr)) { r.Failures.Add(path + ": reverse alias"); continue; }
                fwd[l] = rr; rev[rr] = l; r.Classes++;
                var rc = (RedBaseClass)rr;
                var lp = lc.GetPropertyNames().OrderBy(x => x, StringComparer.OrdinalIgnoreCase).ToList();
                var rp = rc.GetPropertyNames().OrderBy(x => x, StringComparer.OrdinalIgnoreCase).ToList();
                if (string.Join("|", lp) != string.Join("|", rp)) { r.Failures.Add(path + ": properties"); continue; }
                foreach (var n in lp) st.Push((lc.GetProperty(n), rc.GetProperty(n), path + "." + n));
            }
            else if (l is IRedBaseHandle lh) st.Push((lh.GetValue(), ((IRedBaseHandle)rr).GetValue(), path + "->"));
            else if (l is IRedArray la)
            {
                var ra = (IRedArray)rr;
                if (la.Count != ra.Count) { r.Failures.Add(path + ": array count"); continue; }
                for (int i = 0; i < la.Count; i++) st.Push((la[i], ra[i], path + "[" + i + "]"));
            }
            else if (l is IRedBufferPointer lb)
            {
                var x = lb.GetValue(); var y = ((IRedBufferPointer)rr).GetValue(); r.Buffers++;
                if (x.Flags != y.Flags || !x.GetBytes().AsSpan().SequenceEqual(y.GetBytes())) r.Failures.Add(path + ": raw buffer");
            }
            else if (!l.Equals(rr)) r.Failures.Add(path + ": scalar " + l + " != " + rr);
        }
        return r;
    }
}
