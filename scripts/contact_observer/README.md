# Evaluation-only full-robot contact observer

Build against the installed Fortress libraries; no package install or physics
changes are needed:

```bash
cmake -S scripts/contact_observer -B /tmp/b2w-contact-observer-build
cmake --build /tmp/b2w-contact-observer-build -j2
ctest --test-dir /tmp/b2w-contact-observer-build --output-on-failure
```

Load `libb2w_contact_observer.so` as world plugin `b2w_evaluation::Observer`
in a private, provenance-checked copy of the original world. The fixed model and
world names are `b2w` and `course_world`. The plugin requests `ContactSensorData`
for every robot collision entity without modifying any shape or physics setting.
It samples every unpaused PostUpdate. Every forbidden pair is retained for the
whole simulator session and repeated in both subsequent contact reports and the
inventory's sticky veto. Only the four exact foot-ground support pairs are exempt.

Private Gazebo transport topics are `/b2w/evaluation/full_contacts` (`Contacts`)
and `/b2w/evaluation/collision_inventory` (`StringMsg`, JSON). Reports at about
20 Hz include empty interval heartbeats; ROS contact Header frame_id carries
`b2w_evaluation_seq_N`. Match sequence and source stamp across both messages.
Inventory includes interval start, sampled steps, actual last support time and
the cumulative veto. These messages may veto an experiment; they must never
construct navigation maps, goals, transforms or control commands.

The inventory must match an independently hashed converted source SDF, including
all 26 collisions. Synthetic ECM tests exercise every reporting/name path plus
transient/lost-publication retention. Positive live foot-ground controls prove
the engine's contact feature is active. They do not prove successful engine
construction/mapping of every non-foot shape. Existing self-collision settings
remain unchanged, so self-intersection coverage is unavailable. Before a motion
trial can claim contact-free completion, require a post-stop report whose covered
interval reaches the accepted trial end timestamp. Freshness alone leaves a tail.

The stationary capture CLI can retain these reports with
`--collision-inventory SOURCE_BOUND_JSON`. A passive capture PASS never
authorizes motion or claims full-route success.
