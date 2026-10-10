# Arm Ethos-U core driver

This directory contains Arm Ethos-U core driver version 0.16.0. Arm
copyright notices and the Apache-2.0 license are retained.

NuttX integration replaces the bare-metal WFE/SEV intrinsics with NuttX
intrinsics, uses the NuttX debug interface, and distinguishes NuttX's
`__KERNEL__` from Linux's when including integer types.

Architecture lower-halves own power, clock, reset, interrupt, cache and DMA
address handling. Applications use the public NuttX AI engine interface, not
these private Arm headers.
