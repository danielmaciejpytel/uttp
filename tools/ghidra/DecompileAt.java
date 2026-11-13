// Disassemble + decompile functions at given RVAs without full auto-analysis (headless).
// Usage (postScript args): <outFile> <rva1> [rva2 ...]   e.g. 0x17DDB4
// @category UTPP
import ghidra.app.cmd.disassemble.DisassembleCommand;
import ghidra.app.cmd.function.CreateFunctionCmd;
import ghidra.app.decompiler.DecompInterface;
import ghidra.app.decompiler.DecompileResults;
import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.listing.Function;
import ghidra.program.model.listing.Instruction;

import java.io.FileWriter;
import java.io.PrintWriter;

public class DecompileAt extends GhidraScript {
    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        try (PrintWriter out = new PrintWriter(new FileWriter(args[0]))) {
            Address base = currentProgram.getImageBase();
            out.println("imageBase=" + base);
            DecompInterface dec = new DecompInterface();
            dec.openProgram(currentProgram);
            for (int i = 1; i < args.length; i++) {
                long rva = Long.decode(args[i]);
                Address a = base.add(rva);
                new DisassembleCommand(a, null, true).applyTo(currentProgram, monitor);
                Function f = getFunctionAt(a);
                if (f == null) {
                    new CreateFunctionCmd(a).applyTo(currentProgram, monitor);
                    f = getFunctionAt(a);
                }
                out.println("==== RVA " + args[i] + " addr " + a + " function " + (f == null ? "<none>" : f.getName() + " body=" + f.getBody().getNumAddresses() + " bytes"));
                if (f == null) continue;
                Instruction ins = getInstructionAt(a);
                int n = 0;
                while (ins != null && f.getBody().contains(ins.getAddress()) && n < 400) {
                    out.println("  " + ins.getAddress() + "  " + ins);
                    ins = ins.getNext();
                    n++;
                }
                DecompileResults r = dec.decompileFunction(f, 120, monitor);
                out.println("---- decompiled ----");
                out.println(r.decompileCompleted() ? r.getDecompiledFunction().getC() : ("FAILED: " + r.getErrorMessage()));
            }
        }
    }
}
