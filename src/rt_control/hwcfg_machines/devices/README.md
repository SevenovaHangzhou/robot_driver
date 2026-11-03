Device model facts. Each fact needs an authority; unknown values stay TBD.

`faults/` holds one `device_fault_table` per drive series: vendor fault codes transcribed from the
vendor manual (meaning is vendor-defined, not derived from the bus protocol), each mapped to one
company DREE event class. Device files reference their table with `fault_table`. The vendor code
itself is reported in `ErrorInfo.detail`; unlisted codes are reported raw as unregistered.
