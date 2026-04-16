## TODO

- Reconcile the interrupt-polarity behavior and comments in `vl53l1x.chip.c`. The simulation currently reports active-low via `GPIO_HV_MUX__CTRL`, but earlier comments and register defaults still mention the post-init active-high interpretation from the ST config patch.
