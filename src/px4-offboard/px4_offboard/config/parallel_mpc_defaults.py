PARALLEL_MPC_DEFAULTS = {
    "enabled": True,
    "num_workers": 6,
    "dt_ctrl": 0.02,
    "deadline_ms": 10,
    "affinity": [],
    "degradation": "hold_last",
    "transport": "shm",
    "acados": {
        "max_iter": 10,
        "warm_start": True,
        "blas_threads": 1,
        "omp_threads": 1,
    },
}

