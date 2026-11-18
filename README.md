# ExVectr Packet library
This library adds simple standardised packets for communication between systems. 
## **This project is under initial development. Things will probably break.**

## Schema-generated packets (ExVectrNetwork)

`schema/*.toml` define the packets of the ExVectrNetwork stack. `tools/gen_packets.py` generates C++ headers (`gen/`) and a pure-Python package (`python/exvectr_packets`), both committed. Decoded values are SI. See `docs/README.md`. The older hand-written headers in `include/` are unchanged.
