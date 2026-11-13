// Find defined strings matching regex patterns and list their references (address, referencing function).
// Usage (postScript args): <outFile> <maxRefsPerString> <regex1> [regex2 ...]   (regex: Java, case-insensitive)
// @category UTPP
import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.data.StringDataInstance;
import ghidra.program.model.listing.Data;
import ghidra.program.model.listing.DataIterator;
import ghidra.program.model.listing.Function;
import ghidra.program.model.symbol.Reference;
import ghidra.program.model.symbol.ReferenceIterator;
import ghidra.program.util.DefinedDataIterator;

import java.io.FileWriter;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Pattern;

public class StringXrefs extends GhidraScript {
    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        int maxRefs = Integer.parseInt(args[1]);
        List<Pattern> pats = new ArrayList<>();
        for (int i = 2; i < args.length; i++) pats.add(Pattern.compile(args[i], Pattern.CASE_INSENSITIVE));
        Address base = currentProgram.getImageBase();
        try (PrintWriter out = new PrintWriter(new FileWriter(args[0]))) {
            out.println("imageBase=" + base + " patterns=" + pats.size());
            int found = 0;
            DataIterator it = DefinedDataIterator.byDataInstance(currentProgram, Data::hasStringValue);
            while (it.hasNext() && !monitor.isCancelled()) {
                Data d = it.next();
                String s = StringDataInstance.getStringDataInstance(d).getStringValue();
                if (s == null) continue;
                boolean hit = false;
                for (Pattern p : pats) if (p.matcher(s).find()) { hit = true; break; }
                if (!hit) continue;
                found++;
                String clean = s.length() > 160 ? s.substring(0, 160) + "..." : s;
                out.println("STR " + d.getAddress() + " rva=0x" + Long.toHexString(d.getAddress().subtract(base)) + " len=" + s.length() + " \"" + clean.replace("\n", "\\n").replace("\r", "\\r") + "\"");
                ReferenceIterator refs = currentProgram.getReferenceManager().getReferencesTo(d.getAddress());
                int n = 0;
                while (refs.hasNext()) {
                    Reference r = refs.next();
                    if (n++ >= maxRefs) { out.println("    ... more refs"); break; }
                    Address from = r.getFromAddress();
                    Function f = getFunctionContaining(from);
                    out.println("    ref " + from + " rva=0x" + Long.toHexString(from.subtract(base)) + " " + r.getReferenceType()
                        + " in " + (f == null ? "<no function>" : f.getName() + "@" + f.getEntryPoint() + " rva=0x" + Long.toHexString(f.getEntryPoint().subtract(base)) + " size=" + f.getBody().getNumAddresses()));
                }
            }
            out.println("matched strings=" + found);
        }
    }
}
