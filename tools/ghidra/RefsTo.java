// List references (from the analysed program) to each given RVA, with the referencing function.
// Usage (postScript args): <outFile> <maxRefs> <rva1> [rva2 ...]   e.g. 0x34310c0
// @category UTPP
import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.listing.Function;
import ghidra.program.model.symbol.Reference;
import ghidra.program.model.symbol.ReferenceIterator;

import java.io.FileWriter;
import java.io.PrintWriter;

public class RefsTo extends GhidraScript {
    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        int maxRefs = Integer.parseInt(args[1]);
        Address base = currentProgram.getImageBase();
        try (PrintWriter out = new PrintWriter(new FileWriter(args[0]))) {
            out.println("imageBase=" + base);
            for (int i = 2; i < args.length; i++) {
                long rva = Long.decode(args[i]);
                Address a = base.add(rva);
                out.println("==== RVA 0x" + Long.toHexString(rva) + " addr " + a);
                ReferenceIterator it = currentProgram.getReferenceManager().getReferencesTo(a);
                int n = 0;
                while (it.hasNext() && n < maxRefs) {
                    Reference r = it.next();
                    Address from = r.getFromAddress();
                    Function f = getFunctionContaining(from);
                    out.println("  ref " + from + " rva=0x" + Long.toHexString(from.subtract(base)) + " " + r.getReferenceType()
                        + (f != null ? " in " + f.getName() + " rva=0x" + Long.toHexString(f.getEntryPoint().subtract(base)) : " in <none>"));
                    n++;
                }
                out.println("  refs listed=" + n);
            }
        }
    }
}
