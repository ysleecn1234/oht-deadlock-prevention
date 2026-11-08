# Segment OCS admission priority and observation

Base: version2, commit 21a4003f3e2936d1d38468af2102646c48dfe6cc.

## Problem and changes

In OCS mode, segment queue order could block a vehicle already admitted by OCS behind another vehicle waiting for that admission to clear. Prefer admitted queue members, preserving their queue order. FCFS and ADAPTIVE selection remain unchanged; physical movement constraints remain enabled.

Preserve the first blocking reason on the forward route instead of overwriting it with a downstream consequence. Record segment merge queues, control claims and ring reservations, and copy merge-holder/expiry mappings when taking snapshots. Label existing dependency-cycle candidates with control/physical class and unloaded-vehicle presence. These labels do not confirm deadlock. Add progress output and checkpoint snapshots.

## Validation already performed locally

Local regression checks covered OCS priority, other merge modes, occupied-node constraints, route cleanup, reason preservation and candidate recovery. Those local test wrappers depend on a separate workspace and are not included here. No new simulation was run while preparing this branch.

For the same first 1800 simulated seconds (1000 OHT, 0.5-second lot interval, seed 1), baseline completed 18 transfers / 1 lot; the modified run completed 123 transfers / 3 lots. The modified run reported no safety-check failures. This is a single scenario, not proof of general safety or throughput.

## Remaining limitations

Severe waiting remains, including observed cycles retaining previous control-point claims while waiting for the next point. Ring-full detour parity is not restored. This change must not be described as a complete deadlock fix. Candidate counts are not confirmed-deadlock counts.

Only three source files and this note are included. Local launchers, simulation outputs, logs and copied input data are excluded.
