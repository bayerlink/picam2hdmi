# The protocol moved

The wire format this tool speaks is **bayerlink**, and its specification,
reference implementation and conformance vectors live in the protocol's own
repository:

    https://github.com/bayerlink/bayerlink

It moved out because the protocol is the reusable thing: this tool is one
encoder of it, an FPGA receiver is one decoder, and neither should own the
contract the other implements. During the brief period this repository held
the draft under the name "rawlink" (magic `RWLK`), nothing had implemented
it; the released protocol is bayerlink v2, magic `BYLK`.
