# Same-batch corrected physical comparison

Corrected update versus both the original best80 and the stock update. Values remain descriptive; one training batch and three paired noise streams.

| Case | Role | Completed steps | End | Observed return | Task sample |
|---|---|---:|---|---:|---|
| before_nominal | before | 500 | episode_time_limit | 305.0597117964444 | True |
| before_noisy_123 | before | 500 | episode_time_limit | 264.20615595033865 | True |
| before_noisy_125 | before | 500 | episode_time_limit | 212.45231173119132 | True |
| before_noisy_127 | before | 458 | grf_penetration | 229.46504507499526 | True |
| after_nominal | after | 500 | episode_time_limit | 255.1863622414535 | True |
| after_noisy_123 | after | 500 | episode_time_limit | 223.88653591272535 | True |
| after_noisy_125 | after | 500 | episode_time_limit | 231.48506787090326 | True |
| after_noisy_127 | after | 500 | episode_time_limit | 267.6200113718894 | True |
| corrected_nominal | corrected | 500 | episode_time_limit | 279.37783550656934 | True |
| corrected_noisy_123 | corrected | 500 | episode_time_limit | 294.38513595361894 | True |
| corrected_noisy_125 | corrected | 500 | episode_time_limit | 277.27345818685427 | True |
| corrected_noisy_127 | corrected | 500 | episode_time_limit | 244.76796439189928 | True |

## corrected_vs_before/nominal

500 matched completed steps. Observed return change: -25.6819; both finite-task samples: True.

| prefix_tracking — corrected minus reference | Difference |
|---|---:|
| knee.actual_target_deg.rmse | -0.109294 |
| knee.actual_target_deg.centered_unit_rms_shape_error | 0.0248819 |
| knee.served_target_deg.rmse | -0.0934309 |
| knee.served_target_deg.centered_unit_rms_shape_error | 0.0258148 |
| ankle.actual_target_deg.rmse | 0.630909 |
| ankle.actual_target_deg.centered_unit_rms_shape_error | 0.0414317 |
| ankle.served_target_deg.rmse | 0.636294 |
| ankle.served_target_deg.centered_unit_rms_shape_error | 0.0437798 |

| window_tracking — corrected minus reference | Difference |
|---|---:|
| knee.actual_target_deg.rmse | -0.132412 |
| knee.actual_target_deg.centered_unit_rms_shape_error | 0.0235108 |
| knee.served_target_deg.rmse | -0.11787 |
| knee.served_target_deg.centered_unit_rms_shape_error | 0.0244406 |
| ankle.actual_target_deg.rmse | 0.666881 |
| ankle.actual_target_deg.centered_unit_rms_shape_error | 0.0362341 |
| ankle.served_target_deg.rmse | 0.67022 |
| ankle.served_target_deg.centered_unit_rms_shape_error | 0.0383676 |

| torque — corrected minus reference | Difference |
|---|---:|
| knee.rms_nm | 0.38025 |
| knee.min_nm | 1.76668 |
| knee.max_nm | 2.59228 |
| knee.raw_minus_lp20_rms_nm | -0.0779222 |
| knee.raw_minus_lp20_normalized_dynamic_rms | -0.010595 |
| knee.lp10_tv_normalized_per_s | 0.00753666 |
| ankle.rms_nm | -0.0180323 |
| ankle.min_nm | 4.82741 |
| ankle.max_nm | 3.29293 |
| ankle.raw_minus_lp20_rms_nm | 0.0362506 |
| ankle.raw_minus_lp20_normalized_dynamic_rms | 0.00949885 |
| ankle.lp10_tv_normalized_per_s | 0.508357 |


## corrected_vs_after/nominal

500 matched completed steps. Observed return change: 24.1915; both finite-task samples: True.

| prefix_tracking — corrected minus reference | Difference |
|---|---:|
| knee.actual_target_deg.rmse | -0.119096 |
| knee.actual_target_deg.centered_unit_rms_shape_error | 0.00711111 |
| knee.served_target_deg.rmse | -0.131447 |
| knee.served_target_deg.centered_unit_rms_shape_error | 0.00526255 |
| ankle.actual_target_deg.rmse | -0.671117 |
| ankle.actual_target_deg.centered_unit_rms_shape_error | -0.0420885 |
| ankle.served_target_deg.rmse | -0.61335 |
| ankle.served_target_deg.centered_unit_rms_shape_error | -0.0376161 |

| window_tracking — corrected minus reference | Difference |
|---|---:|
| knee.actual_target_deg.rmse | -0.0843418 |
| knee.actual_target_deg.centered_unit_rms_shape_error | 0.00984312 |
| knee.served_target_deg.rmse | -0.106376 |
| knee.served_target_deg.centered_unit_rms_shape_error | 0.00710178 |
| ankle.actual_target_deg.rmse | -0.679652 |
| ankle.actual_target_deg.centered_unit_rms_shape_error | -0.0431625 |
| ankle.served_target_deg.rmse | -0.621353 |
| ankle.served_target_deg.centered_unit_rms_shape_error | -0.0391137 |

| torque — corrected minus reference | Difference |
|---|---:|
| knee.rms_nm | -2.67028 |
| knee.min_nm | 2.35618 |
| knee.max_nm | -10.1905 |
| knee.raw_minus_lp20_rms_nm | 0.275702 |
| knee.raw_minus_lp20_normalized_dynamic_rms | 0.0521442 |
| knee.lp10_tv_normalized_per_s | 0.200189 |
| ankle.rms_nm | -3.42956 |
| ankle.min_nm | -1.78094 |
| ankle.max_nm | -3.95986 |
| ankle.raw_minus_lp20_rms_nm | 0.0197322 |
| ankle.raw_minus_lp20_normalized_dynamic_rms | 0.0243874 |
| ankle.lp10_tv_normalized_per_s | 0.765214 |


## corrected_vs_before/noisy_seed123

500 matched completed steps. Observed return change: 30.179; both finite-task samples: True.

| prefix_tracking — corrected minus reference | Difference |
|---|---:|
| knee.actual_target_deg.rmse | -1.40194 |
| knee.actual_target_deg.centered_unit_rms_shape_error | -0.14434 |
| knee.served_target_deg.rmse | -1.41892 |
| knee.served_target_deg.centered_unit_rms_shape_error | -0.1441 |
| ankle.actual_target_deg.rmse | -0.108452 |
| ankle.actual_target_deg.centered_unit_rms_shape_error | -0.00248872 |
| ankle.served_target_deg.rmse | -0.138325 |
| ankle.served_target_deg.centered_unit_rms_shape_error | -0.00849004 |

| window_tracking — corrected minus reference | Difference |
|---|---:|
| knee.actual_target_deg.rmse | -1.41041 |
| knee.actual_target_deg.centered_unit_rms_shape_error | -0.14816 |
| knee.served_target_deg.rmse | -1.42927 |
| knee.served_target_deg.centered_unit_rms_shape_error | -0.148305 |
| ankle.actual_target_deg.rmse | -0.110666 |
| ankle.actual_target_deg.centered_unit_rms_shape_error | -0.00232114 |
| ankle.served_target_deg.rmse | -0.144275 |
| ankle.served_target_deg.centered_unit_rms_shape_error | -0.00934716 |

| torque — corrected minus reference | Difference |
|---|---:|
| knee.rms_nm | 5.69004 |
| knee.min_nm | -2.47468 |
| knee.max_nm | 13.24 |
| knee.raw_minus_lp20_rms_nm | -0.117814 |
| knee.raw_minus_lp20_normalized_dynamic_rms | -0.0361822 |
| knee.lp10_tv_normalized_per_s | -1.73658 |
| ankle.rms_nm | 1.20684 |
| ankle.min_nm | -7.70181 |
| ankle.max_nm | -14.1626 |
| ankle.raw_minus_lp20_rms_nm | 0.051121 |
| ankle.raw_minus_lp20_normalized_dynamic_rms | 0.00135038 |
| ankle.lp10_tv_normalized_per_s | 0.292414 |


## corrected_vs_after/noisy_seed123

500 matched completed steps. Observed return change: 70.4986; both finite-task samples: True.

| prefix_tracking — corrected minus reference | Difference |
|---|---:|
| knee.actual_target_deg.rmse | -0.902331 |
| knee.actual_target_deg.centered_unit_rms_shape_error | -0.0963979 |
| knee.served_target_deg.rmse | -0.958536 |
| knee.served_target_deg.centered_unit_rms_shape_error | -0.100919 |
| ankle.actual_target_deg.rmse | -0.33321 |
| ankle.actual_target_deg.centered_unit_rms_shape_error | 0.0439964 |
| ankle.served_target_deg.rmse | -0.379484 |
| ankle.served_target_deg.centered_unit_rms_shape_error | 0.0332581 |

| window_tracking — corrected minus reference | Difference |
|---|---:|
| knee.actual_target_deg.rmse | -0.858539 |
| knee.actual_target_deg.centered_unit_rms_shape_error | -0.0930909 |
| knee.served_target_deg.rmse | -0.918761 |
| knee.served_target_deg.centered_unit_rms_shape_error | -0.0981237 |
| ankle.actual_target_deg.rmse | -0.347913 |
| ankle.actual_target_deg.centered_unit_rms_shape_error | 0.0543337 |
| ankle.served_target_deg.rmse | -0.393691 |
| ankle.served_target_deg.centered_unit_rms_shape_error | 0.0440662 |

| torque — corrected minus reference | Difference |
|---|---:|
| knee.rms_nm | 2.10735 |
| knee.min_nm | -7.92267 |
| knee.max_nm | -3.25201 |
| knee.raw_minus_lp20_rms_nm | 0.00812321 |
| knee.raw_minus_lp20_normalized_dynamic_rms | -0.00849544 |
| knee.lp10_tv_normalized_per_s | 0.137495 |
| ankle.rms_nm | -2.08931 |
| ankle.min_nm | -11.7049 |
| ankle.max_nm | -23.3419 |
| ankle.raw_minus_lp20_rms_nm | 0.0657842 |
| ankle.raw_minus_lp20_normalized_dynamic_rms | 0.0105215 |
| ankle.lp10_tv_normalized_per_s | 0.846428 |


## corrected_vs_before/noisy_seed125

500 matched completed steps. Observed return change: 64.8211; both finite-task samples: True.

| prefix_tracking — corrected minus reference | Difference |
|---|---:|
| knee.actual_target_deg.rmse | -3.15306 |
| knee.actual_target_deg.centered_unit_rms_shape_error | -0.160783 |
| knee.served_target_deg.rmse | -3.13865 |
| knee.served_target_deg.centered_unit_rms_shape_error | -0.157394 |
| ankle.actual_target_deg.rmse | -0.528256 |
| ankle.actual_target_deg.centered_unit_rms_shape_error | -0.0181538 |
| ankle.served_target_deg.rmse | -0.464293 |
| ankle.served_target_deg.centered_unit_rms_shape_error | -0.0128894 |

| window_tracking — corrected minus reference | Difference |
|---|---:|
| knee.actual_target_deg.rmse | -3.24055 |
| knee.actual_target_deg.centered_unit_rms_shape_error | -0.165043 |
| knee.served_target_deg.rmse | -3.22086 |
| knee.served_target_deg.centered_unit_rms_shape_error | -0.161212 |
| ankle.actual_target_deg.rmse | -0.536916 |
| ankle.actual_target_deg.centered_unit_rms_shape_error | -0.0224558 |
| ankle.served_target_deg.rmse | -0.468841 |
| ankle.served_target_deg.centered_unit_rms_shape_error | -0.0164535 |

| torque — corrected minus reference | Difference |
|---|---:|
| knee.rms_nm | 1.51469 |
| knee.min_nm | 17.3167 |
| knee.max_nm | 28.2213 |
| knee.raw_minus_lp20_rms_nm | 0.0861611 |
| knee.raw_minus_lp20_normalized_dynamic_rms | -0.00703544 |
| knee.lp10_tv_normalized_per_s | -0.205148 |
| ankle.rms_nm | -1.00266 |
| ankle.min_nm | 5.14718 |
| ankle.max_nm | -29.7799 |
| ankle.raw_minus_lp20_rms_nm | -0.00494412 |
| ankle.raw_minus_lp20_normalized_dynamic_rms | 0.00444343 |
| ankle.lp10_tv_normalized_per_s | -0.403974 |


## corrected_vs_after/noisy_seed125

500 matched completed steps. Observed return change: 45.7884; both finite-task samples: True.

| prefix_tracking — corrected minus reference | Difference |
|---|---:|
| knee.actual_target_deg.rmse | -3.39781 |
| knee.actual_target_deg.centered_unit_rms_shape_error | -0.196104 |
| knee.served_target_deg.rmse | -3.38242 |
| knee.served_target_deg.centered_unit_rms_shape_error | -0.193008 |
| ankle.actual_target_deg.rmse | -0.616639 |
| ankle.actual_target_deg.centered_unit_rms_shape_error | -0.0592903 |
| ankle.served_target_deg.rmse | -0.59082 |
| ankle.served_target_deg.centered_unit_rms_shape_error | -0.0569373 |

| window_tracking — corrected minus reference | Difference |
|---|---:|
| knee.actual_target_deg.rmse | -3.47251 |
| knee.actual_target_deg.centered_unit_rms_shape_error | -0.199483 |
| knee.served_target_deg.rmse | -3.45379 |
| knee.served_target_deg.centered_unit_rms_shape_error | -0.196093 |
| ankle.actual_target_deg.rmse | -0.649263 |
| ankle.actual_target_deg.centered_unit_rms_shape_error | -0.063979 |
| ankle.served_target_deg.rmse | -0.615411 |
| ankle.served_target_deg.centered_unit_rms_shape_error | -0.0606695 |

| torque — corrected minus reference | Difference |
|---|---:|
| knee.rms_nm | 2.89505 |
| knee.min_nm | -2.0287 |
| knee.max_nm | 22.3983 |
| knee.raw_minus_lp20_rms_nm | -0.184275 |
| knee.raw_minus_lp20_normalized_dynamic_rms | -0.0340022 |
| knee.lp10_tv_normalized_per_s | -0.956993 |
| ankle.rms_nm | 0.544854 |
| ankle.min_nm | 3.43217 |
| ankle.max_nm | -9.59506 |
| ankle.raw_minus_lp20_rms_nm | -0.030741 |
| ankle.raw_minus_lp20_normalized_dynamic_rms | -0.00416659 |
| ankle.lp10_tv_normalized_per_s | -0.128001 |


## corrected_vs_before/noisy_seed127

458 matched completed steps. Observed return change: 15.3029; both finite-task samples: True.

| prefix_tracking — corrected minus reference | Difference |
|---|---:|
| knee.actual_target_deg.rmse | -0.399805 |
| knee.actual_target_deg.centered_unit_rms_shape_error | -0.0606339 |
| knee.served_target_deg.rmse | -0.357852 |
| knee.served_target_deg.centered_unit_rms_shape_error | -0.0571768 |
| ankle.actual_target_deg.rmse | -0.0681803 |
| ankle.actual_target_deg.centered_unit_rms_shape_error | -0.0363441 |
| ankle.served_target_deg.rmse | -0.222771 |
| ankle.served_target_deg.centered_unit_rms_shape_error | -0.0534466 |

| window_tracking — corrected minus reference | Difference |
|---|---:|
| knee.actual_target_deg.rmse | -0.418726 |
| knee.actual_target_deg.centered_unit_rms_shape_error | -0.0614601 |
| knee.served_target_deg.rmse | -0.372358 |
| knee.served_target_deg.centered_unit_rms_shape_error | -0.0579215 |
| ankle.actual_target_deg.rmse | -0.0541634 |
| ankle.actual_target_deg.centered_unit_rms_shape_error | -0.0367899 |
| ankle.served_target_deg.rmse | -0.174915 |
| ankle.served_target_deg.centered_unit_rms_shape_error | -0.0492874 |

| torque — corrected minus reference | Difference |
|---|---:|
| knee.rms_nm | -0.0799986 |
| knee.min_nm | -1.39707 |
| knee.max_nm | -2.15164 |
| knee.raw_minus_lp20_rms_nm | 0.169401 |
| knee.raw_minus_lp20_normalized_dynamic_rms | 0.00705703 |
| knee.lp10_tv_normalized_per_s | 0.881264 |
| ankle.rms_nm | -1.05759 |
| ankle.min_nm | 13.2104 |
| ankle.max_nm | 9.22062 |
| ankle.raw_minus_lp20_rms_nm | -0.0434398 |
| ankle.raw_minus_lp20_normalized_dynamic_rms | -0.00224667 |
| ankle.lp10_tv_normalized_per_s | 0.0872146 |


## corrected_vs_after/noisy_seed127

500 matched completed steps. Observed return change: -22.852; both finite-task samples: True.

| prefix_tracking — corrected minus reference | Difference |
|---|---:|
| knee.actual_target_deg.rmse | 0.173094 |
| knee.actual_target_deg.centered_unit_rms_shape_error | 0.0187954 |
| knee.served_target_deg.rmse | 0.160983 |
| knee.served_target_deg.centered_unit_rms_shape_error | 0.0174832 |
| ankle.actual_target_deg.rmse | 0.142452 |
| ankle.actual_target_deg.centered_unit_rms_shape_error | -0.0148795 |
| ankle.served_target_deg.rmse | 0.135842 |
| ankle.served_target_deg.centered_unit_rms_shape_error | -0.0145653 |

| window_tracking — corrected minus reference | Difference |
|---|---:|
| knee.actual_target_deg.rmse | 0.179539 |
| knee.actual_target_deg.centered_unit_rms_shape_error | 0.0200103 |
| knee.served_target_deg.rmse | 0.172183 |
| knee.served_target_deg.centered_unit_rms_shape_error | 0.0192142 |
| ankle.actual_target_deg.rmse | 0.141907 |
| ankle.actual_target_deg.centered_unit_rms_shape_error | -0.0115907 |
| ankle.served_target_deg.rmse | 0.139494 |
| ankle.served_target_deg.centered_unit_rms_shape_error | -0.00983438 |

| torque — corrected minus reference | Difference |
|---|---:|
| knee.rms_nm | 0.034355 |
| knee.min_nm | -8.00038 |
| knee.max_nm | 4.76746 |
| knee.raw_minus_lp20_rms_nm | 0.0372624 |
| knee.raw_minus_lp20_normalized_dynamic_rms | 0.0008083 |
| knee.lp10_tv_normalized_per_s | 0.513579 |
| ankle.rms_nm | -0.500776 |
| ankle.min_nm | -4.53904 |
| ankle.max_nm | -4.6374 |
| ankle.raw_minus_lp20_rms_nm | -0.018012 |
| ankle.raw_minus_lp20_normalized_dynamic_rms | -0.00067321 |
| ankle.lp10_tv_normalized_per_s | 0.0886375 |


- One captured PPO batch with exact control replay and one corrected update does not establish convergence, learning from scratch or generalization.
- Each policy uses its own state-dependent sigma. Shared standard-normal draws do not hold realized action noise or physical states fixed.
- Nominal deployment and stochastic RLlib connector protocols are reported separately.
- True physical early terminations remain in finite-task return summaries. Operational/external truncations retain raw outcomes but are excluded from task-return means.
- Kinematic and torque metrics compare a common completed prefix/window within each pair. Different seeds can have different supported windows; those shape metrics are not pooled as if support were identical.
- Torque similarity measures waveform change relative to before/stock, not biomechanical correctness or V26 torque-target agreement.
- Three paired Gaussian seeds describe finite-sample dispersion only; no p-values or significance claim.
