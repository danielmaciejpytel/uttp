// WkitBridge import <raw.anims.glb> <base.anims> <outDir> [--force-compressed] [--game <dir>]
// Port of Build\test-data\ReloadPhaseProbe\import-anims-api.ps1 (the project's earlier working glTF -> .anims path):
// copies <base.anims> into <outDir> under the GLB's stem name and lets WolvenKit's ImportTask update the clips with
// matching names (new names are added). Game archives are only loaded for reading (rig lookup).
using System.Reflection;
using System.Runtime.Loader;

static class Program
{
    static string WkDir;

    // Project folder: UTPP_PROJECT_ROOT, else the nearest parent of the executable that contains Tools\WolvenKit.CLI.
    static string FindRoot()
    {
        var env = Environment.GetEnvironmentVariable("UTPP_PROJECT_ROOT");
        if (!string.IsNullOrEmpty(env)) return env.TrimEnd('\\') + "\\";
        for (var d = new DirectoryInfo(AppContext.BaseDirectory); d != null; d = d.Parent)
            if (Directory.Exists(Path.Combine(d.FullName, "Tools", "WolvenKit.CLI"))) return d.FullName.TrimEnd('\\') + "\\";
        throw new Exception("project folder not found: set UTPP_PROJECT_ROOT");
    }

    // Game folder: UTPP_GAME_DIR, else "game_dir" in Source\tools\local_paths.json (read only).
    static string FindGame(string root)
    {
        var env = Environment.GetEnvironmentVariable("UTPP_GAME_DIR");
        if (!string.IsNullOrEmpty(env)) return env;
        var cfg = Path.Combine(root, "Source", "tools", "local_paths.json");
        if (File.Exists(cfg))
        {
            using var doc = System.Text.Json.JsonDocument.Parse(File.ReadAllText(cfg));
            if (doc.RootElement.TryGetProperty("game_dir", out var v) && !string.IsNullOrEmpty(v.GetString())) return v.GetString();
        }
        throw new Exception("game folder not configured: pass --game, set UTPP_GAME_DIR or game_dir in Source\\tools\\local_paths.json");
    }

    static int Main(string[] args)
    {
        if (args.Length < 4 || args[0] != "import")
        {
            Console.Error.WriteLine("usage: WkitBridge import <raw.anims.glb> <base.anims> <outDir> [--force-compressed] [--game <dir>]");
            return 160;
        }
        string raw = Path.GetFullPath(args[1]), baseAnims = Path.GetFullPath(args[2]), outDir = Path.GetFullPath(args[3]);
        bool force = args.Contains("--force-compressed");
        string root = FindRoot();
        WkDir = root + @"Tools\WolvenKit.CLI\";
        int gi = Array.IndexOf(args, "--game");
        string game = gi > 0 ? args[gi + 1] : FindGame(root);
        if (!outDir.StartsWith(root, StringComparison.OrdinalIgnoreCase)) throw new Exception("outDir outside the project");
        if (Directory.Exists(outDir)) throw new Exception("outDir exists; preserve prior attempt");

        AssemblyLoadContext.Default.Resolving += (ctx, name) =>
        {
            var p = Path.Combine(WkDir, name.Name + ".dll");
            return File.Exists(p) ? ctx.LoadFromAssemblyPath(p) : null;
        };
        AppContext.SetData("APP_CONTEXT_BASE_DIRECTORY", WkDir);
        Directory.SetCurrentDirectory(WkDir.TrimEnd('\\'));
        var asm = AssemblyLoadContext.Default.LoadFromAssemblyPath(Path.Combine(WkDir, "WolvenKit.CLI.dll"));
        var builder = asm.GetType("WolvenKit.CLI.GenericHost", true).GetMethod("CreateHostBuilder").Invoke(null, new object[] { Array.Empty<string>() });
        var host = builder.GetType().GetMethod("Build").Invoke(builder, null);
        try
        {
            var services = (IServiceProvider)host.GetType().GetProperty("Services").GetValue(host);
            var cfType = AppDomain.CurrentDomain.GetAssemblies().Select(a => a.GetType("CP77Tools.Tasks.ConsoleFunctions", false)).First(t => t != null);
            var cf = services.GetService(cfType);
            var bf = BindingFlags.NonPublic | BindingFlags.Instance;
            var manager = cfType.GetField("_archiveManager", bf).GetValue(cf);
            var options = cfType.GetField("_gltfImportArgs", bf).GetValue(cf);
            var val = options.GetType().GetProperty("Value").GetValue(options);
            var fmtProp = val.GetType().GetProperty("ImportFormat");
            fmtProp.SetValue(val, Enum.Parse(fmtProp.PropertyType, "Anims"));
            val.GetType().GetProperty("Keep").SetValue(val, true);
            if (force) val.GetType().GetProperty("ForceCompressedEncoding").SetValue(val, true);

            var oodleType = AppDomain.CurrentDomain.GetAssemblies().Select(a => a.GetType("WolvenKit.Core.Compression.Oodle", false)).First(t => t != null);
            var load = oodleType.GetMethods().First(m => m.Name == "Load");
            var largs = load.GetParameters().Select(p => p.HasDefaultValue ? p.DefaultValue : null).ToArray();
            Console.WriteLine("OODLE_LOAD=" + load.Invoke(null, largs));

            var loadArchive = manager.GetType().GetMethod("LoadArchive");
            var kind = Enum.Parse(loadArchive.GetParameters()[1].ParameterType, "Base");
            foreach (var a in new[] { @"archive\pc\content\basegame_1_engine.archive", @"archive\pc\content\basegame_4_animation.archive" })
                loadArchive.Invoke(manager, new object[] { Path.Combine(game, a), kind });

            Directory.CreateDirectory(outDir);
            var stem = Path.GetFileNameWithoutExtension(raw);  // xxx.anims
            if (!stem.EndsWith(".anims")) throw new Exception("expected <name>.anims.glb");
            var dst = Path.Combine(outDir, stem);
            File.Copy(baseAnims, dst);
            File.SetAttributes(dst, FileAttributes.Normal);
            var task = cfType.GetMethod("ImportTask").Invoke(cf, new object[] { new FileSystemInfo[] { new FileInfo(raw) }, new DirectoryInfo(outDir), true });
            var result = (int)task.GetType().GetProperty("Result").GetValue(task);
            Console.WriteLine("ANIMS_IMPORT_EXIT=" + result);
            return result;
        }
        finally { ((IDisposable)host).Dispose(); }
    }
}
