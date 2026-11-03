These are the WingLoop changes I'd make, most useful first, based on what this project has run into.
  One expectation to set first: ASWING alone takes 0.086 s/step against WingLoop's 0.107, and at 20
  workers it does 73 steps/s against 52–63. So WingLoop changes can recover roughly 20–40 % of
  throughput, not more; most of the remaining cost is ASWING writing a full point file every step.

  A. Throughput

  1. Stop busy-waiting on point files. The wait loop uses time.sleep(1e-8), which keeps one CPU thread
     spinning per worker; with 20 workers that is 20 threads taken from ASWING. Use inotify, or poll
     every 1–2 ms. aswing_raw.RawAswing.wait_file polls at 2 ms with no measurable delay. This is the
     single biggest WingLoop-side gain.
  2. Add a fast path for open-loop runs. When the controller doesn't read the state (fixed stick, or
     excitation signals known in advance), send the commands of many steps ahead (x, Dt ±1, any
     flap/engine changes, W s<k>). Then read the point files as they appear, as aswing_raw does with its
     feeder thread. Keep the per-step loop only for real closed loop. The excitation controller is
     open-loop except for recovery, so a run could use the fast path until the recovery condition, or
     accept a short lag in the recovery check.
  3. Parse less, and delete point files once read. Read only the header channels needed plus the state
     block, compile the regexes once, and delete each file after parsing (the raw driver does this).
     Large point-file directories slow everything down. NOT_THIS
  4. Store output in a compact format. Today a 100 s run makes a .json.gz of about 160 MB, while the .npz
     holding the same information is about 1 MB. Write numeric arrays to .npz/HDF5, with the state
     history as one float32 array. {Optionally keep only selected parts of ModelStates (the joint block,
     the PARAM "additional variables"), set by an option.NOT_THIS}
  5. Limit threads per worker. Set OMP_NUM_THREADS=1 / OPENBLAS_NUM_THREADS=1 in the worker environment
     so numpy doesn't oversubscribe the machine.

  B. Correctness and data integrity

  6. Make the end-marker wait the only point-file read. Read a point file only after it ends with
     ENDEDPRINTINGSTATES, everywhere, including any path other than
     send_writefile_command_and_receive_old. Truncated reads under parallel load caused false "not
     converged" steps, then lost and duplicated samples (2026-11-01 audit).
  7. Keep the point-file timeout configurable. Keep write_timeout (in 36b7dd7) as a constructor argument
     with a large default (≥ 1200 s). Slow converging stall steps took 38–45 s each when run alone, and
     over 100 s under load.
  8. Record a clean time grid. Record every step from Dt to N·Dt; today the first chunk is dropped and
     the run ends at (N−1)·Dt. Optionally record the trim state at t = 0 too. Write time at full
     precision: point files print 4 decimals, so rom_data rebuilds the grid.
  9. Make the step retry explicit. Today WingLoop's 10-sub-step retry on a non-converged step is silent.
     Make it an option, log every retry (step, sub-steps used), and never produce lost or duplicated
     samples. ASWING now sub-steps failed steps itself (TSPLIT), so the WingLoop retry could default to
     off, with the step reported as not converged.
  10. Record per-step solver diagnostics. Record IsConverged, plus the Newton restart and TSPLIT counts
      read from the log, as channels.
  11. Keep the Peng fix. Take the first Peng k match per point file (in 36b7dd7), and add a test, since
      that bug doubled the E1/E2 arrays.

  C. Features this project needs

  12. User stop condition. Accept a callback stop(state) -> bool, e.g. |α| > 105° or divergence, so a run
      ends early instead of marching through data that will be thrown away.
  13. Built-in aero-flag check. After OPER, set and read back the k-menu toggles (frozen VL, lag off) and
      raise an error if they're wrong. The lag setting isn't stored in .set files. This is what
      aswing_runs.set_aero_flags does today.
  14. Hang handling. Put a watchdog on every command, and on a hang kill ASWING's process group and
      return a clear status ("hung at step k").
  15. Command logging in the controller interface. Log the applied command per step natively (time,
      F1–F4, E1, E2, plus any user flags such as recovery). Today excitation_controller does this itself.
  16. Named channels for alpha_ref, beta_ref (PARAM 70–71). They are currently dug out of the "additional
      variables" part of ModelStates.
  17. Run metadata. Write the ASWING binary path and hash, the WingLoop git commit, Dt, N, K, the flags
      and the wall time into the output.

  D. Usability

  18. Explicit ASWING executable path. Accept the path directly, with alias resolution only as a
      fallback. Resolving the alias through bash -ic in worker processes gets them stopped by SIGTTOU.
  19. Headless by default. No live plot and no matplotlib import unless it's asked for.
  20. Filename guard. Raise an error on file names over 80 characters, since ASWING truncates them.
  21. Packaging. Make it installable in any env with pip install -e, with declared dependencies. It
      currently imports only in Matlab on the 7550.

{NOT_THIS  Not WingLoop, but the next big lever

  The largest remaining cost is on the ASWING side. A lighter per-step output command (header channels,
  joint block and PARAM extras only, or a binary dump) could cut much of ASWING's per-step write time.
  That would need a change in ASWING_Modified.}


