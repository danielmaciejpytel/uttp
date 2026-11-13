// Report the function containing each RVA: name, entry, size, callers (count + first N), callees (first N), and the first instructions.
// Usage (postScript args): <outFile> <maxList> <rva1> [rva2 ...]   e.g. 0x52932A
// @category UTPP
import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.listing.Function;
import ghidra.program.model.listing.Instruction;
import ghidra.program.model.symbol.Reference;
import ghidra.program.model.symbol.ReferenceIterator;

import java.io.FileWriter;
import java.io.PrintWriter;
import java.util.Set;

public class FuncInfo extends GhidraScript {
    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        int max = Integer.parseInt(args[1]);
        Address base = currentProgram.getImageBase();
        try (PrintWriter out = new PrintWriter(new FileWriter(args[0]))) {
            out.println("imageBase=" + base);
            for (int i = 2; i < args.length; i++) {
                long rva = Long.decode(args[i]);
                Address a = base.add(rva);
                Function f = getFunctionContaining(a);
                out.println("==== RVA " + args[i] + " addr " + a + (f == null ? " <no function>" : ""));
                Instruction ins = getInstructionAt(a);
                out.println("  insn: " + (ins == null ? "<undefined>" : ins.toString()));
                if (f == null) continue;
                out.println("  function " + f.getName() + " entry " + f.getEntryPoint() + " rva=0x" + Long.toHexString(f.getEntryPoint().subtract(base))
                    + " size=" + f.getBody().getNumAddresses() + " sig=" + f.getSignature());
                ReferenceIterator refs = currentProgram.getReferenceManager().getReferencesTo(f.getEntryPoint());
                int n = 0, total = 0;
                StringBuilder callers = new StringBuilder();
                while (refs.hasNext()) {
                    Reference r = refs.next();
                    total++;
                    if (n < max) {
                        Function c = getFunctionContaining(r.getFromAddress());
                        callers.append("    caller ").append(r.getFromAddress()).append(" ").append(r.getReferenceType())
                            .append(" in ").append(c == null ? "<none>" : c.getName() + " rva=0x" + Long.toHexString(c.getEntryPoint().subtract(base))).append("\n");
                        n++;
                    }
                }
                out.println("  references to entry: " + total);
                out.print(callers);
                Set<Function> callees = f.getCalledFunctions(monitor);
                out.println("  callees: " + callees.size());
                int k = 0;
                for (Function c : callees) { if (k++ >= max) break; out.println("    callee " + c.getName() + " rva=0x" + Long.toHexString(c.getEntryPoint().subtract(base))); }
                Instruction x = getInstructionAt(f.getEntryPoint());
                int m = 0;
                while (x != null && f.getBody().contains(x.getAddress()) && m++ < 12) { out.println("    " + x.getAddress() + "  " + x); x = x.getNext(); }
            }
        }
    }
}
