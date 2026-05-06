# Sweep Tilt Repeatability Results

Generated from report CSVs in `runs/analysis_sweep_tilt_repeatability`.

## Overall (All Trials)

| dataset | n_trials | mean_l2_m | mean_abs_x_m | mean_abs_y_m | mean_abs_z_m | mean_ang_rad |
|---|---|---|---|---|---|---|
| real | 96 | 0.034884 | 0.016633 | 0.013163 | 0.026202 | 0.11802 |
| sim | 96 | 0.0088208 | 0.0067291 | 0.00084469 | 0.0055391 | 0.00053407 |

## By Arm (All Trials)

| dataset | arm | n_trials | mean_l2_m | mean_abs_x_m | mean_abs_y_m | mean_abs_z_m | mean_ang_rad |
|---|---|---|---|---|---|---|---|
| real | left | 48 | 0.030986 | 0.012873 | 0.0085131 | 0.025847 | 0.13926 |
| real | right | 48 | 0.038782 | 0.020393 | 0.017814 | 0.026556 | 0.096775 |
| sim | left | 48 | 0.0088338 | 0.0067392 | 0.00085 | 0.0055465 | 0.00053486 |
| sim | right | 48 | 0.0088077 | 0.006719 | 0.00083938 | 0.0055317 | 0.00053329 |

## By Case (All Trials)

| dataset | case | arm | frame | n_trials | mean_l2_m | mean_abs_x_m | mean_abs_y_m | mean_abs_z_m | mean_ang_rad |
|---|---|---|---|---|---|---|---|---|---|
| real | left_pick_x30_y20_z06 | left | left_wrist_yaw_link | 3 | 0.025951 | 0.011827 | 0.0049667 | 0.022557 | 0.096155 |
| real | left_pick_x30_y20_z10 | left | left_wrist_yaw_link | 3 | 0.0322 | 0.015883 | 0.00638 | 0.027273 | 0.56088 |
| real | left_pick_x30_y20_z14 | left | left_wrist_yaw_link | 3 | 0.034779 | 0.01189 | 0.017677 | 0.02749 | 0.52296 |
| real | left_pick_x30_y20_z18 | left | left_wrist_yaw_link | 3 | 0.038645 | 0.0104 | 0.023977 | 0.028463 | 0.28253 |
| real | left_pick_x30_y30_z06 | left | left_wrist_yaw_link | 3 | 0.019409 | 0.00693 | 0.0014667 | 0.018067 | 0.028343 |
| real | left_pick_x30_y30_z10 | left | left_wrist_yaw_link | 3 | 0.01534 | 0.0015067 | 0.00093 | 0.015203 | 0.035409 |
| real | left_pick_x30_y30_z14 | left | left_wrist_yaw_link | 3 | 0.012074 | 0.0052467 | 0.0014433 | 0.010777 | 0.025017 |
| real | left_pick_x30_y30_z18 | left | left_wrist_yaw_link | 3 | 0.014988 | 0.0043233 | 0.00361 | 0.013877 | 0.029788 |
| real | left_pick_x35_y20_z06 | left | left_wrist_yaw_link | 3 | 0.038957 | 0.022103 | 0.012027 | 0.029213 | 0.029279 |
| real | left_pick_x35_y20_z10 | left | left_wrist_yaw_link | 3 | 0.039705 | 0.01948 | 0.012277 | 0.032047 | 0.065972 |
| real | left_pick_x35_y20_z14 | left | left_wrist_yaw_link | 3 | 0.037556 | 0.016407 | 0.010517 | 0.031767 | 0.25305 |
| real | left_pick_x35_y20_z18 | left | left_wrist_yaw_link | 3 | 0.040705 | 0.01589 | 0.010693 | 0.035463 | 0.087152 |
| real | left_pick_x35_y30_z06 | left | left_wrist_yaw_link | 3 | 0.04739 | 0.025263 | 0.01884 | 0.03322 | 0.055462 |
| real | left_pick_x35_y30_z10 | left | left_wrist_yaw_link | 3 | 0.03904 | 0.02088 | 0.00537 | 0.032453 | 0.046012 |
| real | left_pick_x35_y30_z14 | left | left_wrist_yaw_link | 3 | 0.031816 | 0.011917 | 0.0028233 | 0.02934 | 0.057355 |
| real | left_pick_x35_y30_z18 | left | left_wrist_yaw_link | 3 | 0.027217 | 0.0060167 | 0.0032133 | 0.026347 | 0.052824 |
| real | right_pick_x30_y20_z06 | right | right_wrist_yaw_link | 3 | 0.043733 | 0.02676 | 0.02272 | 0.026063 | 0.092966 |
| real | right_pick_x30_y20_z10 | right | right_wrist_yaw_link | 3 | 0.039656 | 0.02137 | 0.01948 | 0.02713 | 0.10928 |
| real | right_pick_x30_y20_z14 | right | right_wrist_yaw_link | 3 | 0.043437 | 0.02066 | 0.026517 | 0.02751 | 0.13489 |
| real | right_pick_x30_y20_z18 | right | right_wrist_yaw_link | 3 | 0.043393 | 0.014307 | 0.03273 | 0.024637 | 0.14308 |
| real | right_pick_x30_y30_z06 | right | right_wrist_yaw_link | 3 | 0.030876 | 0.018567 | 0.012503 | 0.02126 | 0.038733 |
| real | right_pick_x30_y30_z10 | right | right_wrist_yaw_link | 3 | 0.02287 | 0.0083967 | 0.018093 | 0.011163 | 0.041351 |
| real | right_pick_x30_y30_z14 | right | right_wrist_yaw_link | 3 | 0.021277 | 0.0080433 | 0.014313 | 0.013527 | 0.066806 |
| real | right_pick_x30_y30_z18 | right | right_wrist_yaw_link | 3 | 0.020878 | 0.0055067 | 0.013687 | 0.014773 | 0.078668 |
| real | right_pick_x35_y20_z06 | right | right_wrist_yaw_link | 3 | 0.046095 | 0.028733 | 0.017483 | 0.031463 | 0.032858 |
| real | right_pick_x35_y20_z10 | right | right_wrist_yaw_link | 3 | 0.042496 | 0.022547 | 0.017223 | 0.03162 | 0.042624 |
| real | right_pick_x35_y20_z14 | right | right_wrist_yaw_link | 3 | 0.048486 | 0.02555 | 0.018757 | 0.03669 | 0.089904 |
| real | right_pick_x35_y20_z18 | right | right_wrist_yaw_link | 3 | 0.043498 | 0.020043 | 0.014043 | 0.03596 | 0.11827 |
| real | right_pick_x35_y30_z06 | right | right_wrist_yaw_link | 3 | 0.049548 | 0.037483 | 0.013797 | 0.029157 | 0.020359 |
| real | right_pick_x35_y30_z10 | right | right_wrist_yaw_link | 3 | 0.042246 | 0.02508 | 0.01463 | 0.030683 | 0.022017 |
| real | right_pick_x35_y30_z14 | right | right_wrist_yaw_link | 3 | 0.044464 | 0.024863 | 0.01531 | 0.033527 | 0.42247 |
| real | right_pick_x35_y30_z18 | right | right_wrist_yaw_link | 3 | 0.037564 | 0.018383 | 0.013733 | 0.02974 | 0.09414 |
| sim | left_pick_x30_y20_z06 | left | left_wrist_yaw_link | 3 | 0 | 0 | 0 | 0 | 3.6732e-06 |
| sim | left_pick_x30_y20_z10 | left | left_wrist_yaw_link | 3 | 0.0028606 | 0.0019567 | 2.3333e-05 | 0.0020867 | 0.00013408 |
| sim | left_pick_x30_y20_z14 | left | left_wrist_yaw_link | 3 | 0.02037 | 0.014843 | 0.00012667 | 0.01395 | 0.0011673 |
| sim | left_pick_x30_y20_z18 | left | left_wrist_yaw_link | 3 | 0.048931 | 0.03809 | 0.00023 | 0.030713 | 0.003062 |
| sim | left_pick_x30_y30_z06 | left | left_wrist_yaw_link | 3 | 0 | 0 | 0 | 0 | 3.6733e-06 |
| sim | left_pick_x30_y30_z10 | left | left_wrist_yaw_link | 3 | 0.00026712 | 0.00018 | 5.3333e-05 | 0.00019 | 1.06e-05 |
| sim | left_pick_x30_y30_z14 | left | left_wrist_yaw_link | 3 | 0.013117 | 0.0093767 | 0.0028567 | 0.0087167 | 0.00073672 |
| sim | left_pick_x30_y30_z18 | left | left_wrist_yaw_link | 3 | 0.038821 | 0.0295 | 0.00912 | 0.02353 | 0.0023802 |
| sim | left_pick_x35_y20_z06 | left | left_wrist_yaw_link | 3 | 0 | 0 | 0 | 0 | 3.6732e-06 |
| sim | left_pick_x35_y20_z10 | left | left_wrist_yaw_link | 3 | 0 | 0 | 0 | 0 | 3.6732e-06 |
| sim | left_pick_x35_y20_z14 | left | left_wrist_yaw_link | 3 | 0.00049352 | 0.00038667 | 0 | 0.00030667 | 2.8423e-05 |
| sim | left_pick_x35_y20_z18 | left | left_wrist_yaw_link | 3 | 0.011228 | 0.0092533 | 7e-05 | 0.00636 | 0.00070372 |
| sim | left_pick_x35_y30_z06 | left | left_wrist_yaw_link | 3 | 0 | 0 | 0 | 0 | 3.6733e-06 |
| sim | left_pick_x35_y30_z10 | left | left_wrist_yaw_link | 3 | 0 | 0 | 0 | 0 | 3.6733e-06 |
| sim | left_pick_x35_y30_z14 | left | left_wrist_yaw_link | 3 | 0 | 0 | 0 | 0 | 3.6733e-06 |
| sim | left_pick_x35_y30_z18 | left | left_wrist_yaw_link | 3 | 0.0052521 | 0.00424 | 0.00112 | 0.00289 | 0.00030893 |
| sim | right_pick_x30_y20_z06 | right | right_wrist_yaw_link | 3 | 0 | 0 | 0 | 0 | 3.6733e-06 |
| sim | right_pick_x30_y20_z10 | right | right_wrist_yaw_link | 3 | 0.0023348 | 0.0015967 | 2e-05 | 0.0017033 | 0.00010894 |
| sim | right_pick_x30_y20_z14 | right | right_wrist_yaw_link | 3 | 0.021882 | 0.015947 | 0.00014333 | 0.014983 | 0.0012533 |
| sim | right_pick_x30_y20_z18 | right | right_wrist_yaw_link | 3 | 0.047858 | 0.03725 | 0.00021 | 0.030047 | 0.0029987 |
| sim | right_pick_x30_y30_z06 | right | right_wrist_yaw_link | 3 | 0 | 0 | 0 | 0 | 3.6732e-06 |
| sim | right_pick_x30_y30_z10 | right | right_wrist_yaw_link | 3 | 0.00026712 | 0.00018 | 5.3333e-05 | 0.00019 | 1.06e-05 |
| sim | right_pick_x30_y30_z14 | right | right_wrist_yaw_link | 3 | 0.013554 | 0.00969 | 0.00295 | 0.0090067 | 0.00076029 |
| sim | right_pick_x30_y30_z18 | right | right_wrist_yaw_link | 3 | 0.03719 | 0.028257 | 0.0087533 | 0.02254 | 0.0022823 |
| sim | right_pick_x35_y20_z06 | right | right_wrist_yaw_link | 3 | 0 | 0 | 0 | 0 | 3.6733e-06 |
| sim | right_pick_x35_y20_z10 | right | right_wrist_yaw_link | 3 | 0 | 0 | 0 | 0 | 3.6732e-06 |
| sim | right_pick_x35_y20_z14 | right | right_wrist_yaw_link | 3 | 0.00049352 | 0.00038667 | 0 | 0.00030667 | 2.5157e-05 |
| sim | right_pick_x35_y20_z18 | right | right_wrist_yaw_link | 3 | 0.011581 | 0.0095433 | 7.3333e-05 | 0.00656 | 0.00072467 |
| sim | right_pick_x35_y30_z06 | right | right_wrist_yaw_link | 3 | 0 | 0 | 0 | 0 | 3.6733e-06 |
| sim | right_pick_x35_y30_z10 | right | right_wrist_yaw_link | 3 | 0 | 0 | 0 | 0 | 3.6733e-06 |
| sim | right_pick_x35_y30_z14 | right | right_wrist_yaw_link | 3 | 0 | 0 | 0 | 0 | 3.6733e-06 |
| sim | right_pick_x35_y30_z18 | right | right_wrist_yaw_link | 3 | 0.0057626 | 0.0046533 | 0.0012267 | 0.00317 | 0.00034285 |

## By Case (real)

| dataset | case | arm | frame | n_trials | mean_l2_m | mean_abs_x_m | mean_abs_y_m | mean_abs_z_m | mean_ang_rad |
|---|---|---|---|---|---|---|---|---|---|
| real | left_pick_x30_y20_z06 | left | left_wrist_yaw_link | 3 | 0.025951 | 0.011827 | 0.0049667 | 0.022557 | 0.096155 |
| real | left_pick_x30_y20_z10 | left | left_wrist_yaw_link | 3 | 0.0322 | 0.015883 | 0.00638 | 0.027273 | 0.56088 |
| real | left_pick_x30_y20_z14 | left | left_wrist_yaw_link | 3 | 0.034779 | 0.01189 | 0.017677 | 0.02749 | 0.52296 |
| real | left_pick_x30_y20_z18 | left | left_wrist_yaw_link | 3 | 0.038645 | 0.0104 | 0.023977 | 0.028463 | 0.28253 |
| real | left_pick_x30_y30_z06 | left | left_wrist_yaw_link | 3 | 0.019409 | 0.00693 | 0.0014667 | 0.018067 | 0.028343 |
| real | left_pick_x30_y30_z10 | left | left_wrist_yaw_link | 3 | 0.01534 | 0.0015067 | 0.00093 | 0.015203 | 0.035409 |
| real | left_pick_x30_y30_z14 | left | left_wrist_yaw_link | 3 | 0.012074 | 0.0052467 | 0.0014433 | 0.010777 | 0.025017 |
| real | left_pick_x30_y30_z18 | left | left_wrist_yaw_link | 3 | 0.014988 | 0.0043233 | 0.00361 | 0.013877 | 0.029788 |
| real | left_pick_x35_y20_z06 | left | left_wrist_yaw_link | 3 | 0.038957 | 0.022103 | 0.012027 | 0.029213 | 0.029279 |
| real | left_pick_x35_y20_z10 | left | left_wrist_yaw_link | 3 | 0.039705 | 0.01948 | 0.012277 | 0.032047 | 0.065972 |
| real | left_pick_x35_y20_z14 | left | left_wrist_yaw_link | 3 | 0.037556 | 0.016407 | 0.010517 | 0.031767 | 0.25305 |
| real | left_pick_x35_y20_z18 | left | left_wrist_yaw_link | 3 | 0.040705 | 0.01589 | 0.010693 | 0.035463 | 0.087152 |
| real | left_pick_x35_y30_z06 | left | left_wrist_yaw_link | 3 | 0.04739 | 0.025263 | 0.01884 | 0.03322 | 0.055462 |
| real | left_pick_x35_y30_z10 | left | left_wrist_yaw_link | 3 | 0.03904 | 0.02088 | 0.00537 | 0.032453 | 0.046012 |
| real | left_pick_x35_y30_z14 | left | left_wrist_yaw_link | 3 | 0.031816 | 0.011917 | 0.0028233 | 0.02934 | 0.057355 |
| real | left_pick_x35_y30_z18 | left | left_wrist_yaw_link | 3 | 0.027217 | 0.0060167 | 0.0032133 | 0.026347 | 0.052824 |
| real | right_pick_x30_y20_z06 | right | right_wrist_yaw_link | 3 | 0.043733 | 0.02676 | 0.02272 | 0.026063 | 0.092966 |
| real | right_pick_x30_y20_z10 | right | right_wrist_yaw_link | 3 | 0.039656 | 0.02137 | 0.01948 | 0.02713 | 0.10928 |
| real | right_pick_x30_y20_z14 | right | right_wrist_yaw_link | 3 | 0.043437 | 0.02066 | 0.026517 | 0.02751 | 0.13489 |
| real | right_pick_x30_y20_z18 | right | right_wrist_yaw_link | 3 | 0.043393 | 0.014307 | 0.03273 | 0.024637 | 0.14308 |
| real | right_pick_x30_y30_z06 | right | right_wrist_yaw_link | 3 | 0.030876 | 0.018567 | 0.012503 | 0.02126 | 0.038733 |
| real | right_pick_x30_y30_z10 | right | right_wrist_yaw_link | 3 | 0.02287 | 0.0083967 | 0.018093 | 0.011163 | 0.041351 |
| real | right_pick_x30_y30_z14 | right | right_wrist_yaw_link | 3 | 0.021277 | 0.0080433 | 0.014313 | 0.013527 | 0.066806 |
| real | right_pick_x30_y30_z18 | right | right_wrist_yaw_link | 3 | 0.020878 | 0.0055067 | 0.013687 | 0.014773 | 0.078668 |
| real | right_pick_x35_y20_z06 | right | right_wrist_yaw_link | 3 | 0.046095 | 0.028733 | 0.017483 | 0.031463 | 0.032858 |
| real | right_pick_x35_y20_z10 | right | right_wrist_yaw_link | 3 | 0.042496 | 0.022547 | 0.017223 | 0.03162 | 0.042624 |
| real | right_pick_x35_y20_z14 | right | right_wrist_yaw_link | 3 | 0.048486 | 0.02555 | 0.018757 | 0.03669 | 0.089904 |
| real | right_pick_x35_y20_z18 | right | right_wrist_yaw_link | 3 | 0.043498 | 0.020043 | 0.014043 | 0.03596 | 0.11827 |
| real | right_pick_x35_y30_z06 | right | right_wrist_yaw_link | 3 | 0.049548 | 0.037483 | 0.013797 | 0.029157 | 0.020359 |
| real | right_pick_x35_y30_z10 | right | right_wrist_yaw_link | 3 | 0.042246 | 0.02508 | 0.01463 | 0.030683 | 0.022017 |
| real | right_pick_x35_y30_z14 | right | right_wrist_yaw_link | 3 | 0.044464 | 0.024863 | 0.01531 | 0.033527 | 0.42247 |
| real | right_pick_x35_y30_z18 | right | right_wrist_yaw_link | 3 | 0.037564 | 0.018383 | 0.013733 | 0.02974 | 0.09414 |

## By Case (sim)

| dataset | case | arm | frame | n_trials | mean_l2_m | mean_abs_x_m | mean_abs_y_m | mean_abs_z_m | mean_ang_rad |
|---|---|---|---|---|---|---|---|---|---|
| sim | left_pick_x30_y20_z06 | left | left_wrist_yaw_link | 3 | 0 | 0 | 0 | 0 | 3.6732e-06 |
| sim | left_pick_x30_y20_z10 | left | left_wrist_yaw_link | 3 | 0.0028606 | 0.0019567 | 2.3333e-05 | 0.0020867 | 0.00013408 |
| sim | left_pick_x30_y20_z14 | left | left_wrist_yaw_link | 3 | 0.02037 | 0.014843 | 0.00012667 | 0.01395 | 0.0011673 |
| sim | left_pick_x30_y20_z18 | left | left_wrist_yaw_link | 3 | 0.048931 | 0.03809 | 0.00023 | 0.030713 | 0.003062 |
| sim | left_pick_x30_y30_z06 | left | left_wrist_yaw_link | 3 | 0 | 0 | 0 | 0 | 3.6733e-06 |
| sim | left_pick_x30_y30_z10 | left | left_wrist_yaw_link | 3 | 0.00026712 | 0.00018 | 5.3333e-05 | 0.00019 | 1.06e-05 |
| sim | left_pick_x30_y30_z14 | left | left_wrist_yaw_link | 3 | 0.013117 | 0.0093767 | 0.0028567 | 0.0087167 | 0.00073672 |
| sim | left_pick_x30_y30_z18 | left | left_wrist_yaw_link | 3 | 0.038821 | 0.0295 | 0.00912 | 0.02353 | 0.0023802 |
| sim | left_pick_x35_y20_z06 | left | left_wrist_yaw_link | 3 | 0 | 0 | 0 | 0 | 3.6732e-06 |
| sim | left_pick_x35_y20_z10 | left | left_wrist_yaw_link | 3 | 0 | 0 | 0 | 0 | 3.6732e-06 |
| sim | left_pick_x35_y20_z14 | left | left_wrist_yaw_link | 3 | 0.00049352 | 0.00038667 | 0 | 0.00030667 | 2.8423e-05 |
| sim | left_pick_x35_y20_z18 | left | left_wrist_yaw_link | 3 | 0.011228 | 0.0092533 | 7e-05 | 0.00636 | 0.00070372 |
| sim | left_pick_x35_y30_z06 | left | left_wrist_yaw_link | 3 | 0 | 0 | 0 | 0 | 3.6733e-06 |
| sim | left_pick_x35_y30_z10 | left | left_wrist_yaw_link | 3 | 0 | 0 | 0 | 0 | 3.6733e-06 |
| sim | left_pick_x35_y30_z14 | left | left_wrist_yaw_link | 3 | 0 | 0 | 0 | 0 | 3.6733e-06 |
| sim | left_pick_x35_y30_z18 | left | left_wrist_yaw_link | 3 | 0.0052521 | 0.00424 | 0.00112 | 0.00289 | 0.00030893 |
| sim | right_pick_x30_y20_z06 | right | right_wrist_yaw_link | 3 | 0 | 0 | 0 | 0 | 3.6733e-06 |
| sim | right_pick_x30_y20_z10 | right | right_wrist_yaw_link | 3 | 0.0023348 | 0.0015967 | 2e-05 | 0.0017033 | 0.00010894 |
| sim | right_pick_x30_y20_z14 | right | right_wrist_yaw_link | 3 | 0.021882 | 0.015947 | 0.00014333 | 0.014983 | 0.0012533 |
| sim | right_pick_x30_y20_z18 | right | right_wrist_yaw_link | 3 | 0.047858 | 0.03725 | 0.00021 | 0.030047 | 0.0029987 |
| sim | right_pick_x30_y30_z06 | right | right_wrist_yaw_link | 3 | 0 | 0 | 0 | 0 | 3.6732e-06 |
| sim | right_pick_x30_y30_z10 | right | right_wrist_yaw_link | 3 | 0.00026712 | 0.00018 | 5.3333e-05 | 0.00019 | 1.06e-05 |
| sim | right_pick_x30_y30_z14 | right | right_wrist_yaw_link | 3 | 0.013554 | 0.00969 | 0.00295 | 0.0090067 | 0.00076029 |
| sim | right_pick_x30_y30_z18 | right | right_wrist_yaw_link | 3 | 0.03719 | 0.028257 | 0.0087533 | 0.02254 | 0.0022823 |
| sim | right_pick_x35_y20_z06 | right | right_wrist_yaw_link | 3 | 0 | 0 | 0 | 0 | 3.6733e-06 |
| sim | right_pick_x35_y20_z10 | right | right_wrist_yaw_link | 3 | 0 | 0 | 0 | 0 | 3.6732e-06 |
| sim | right_pick_x35_y20_z14 | right | right_wrist_yaw_link | 3 | 0.00049352 | 0.00038667 | 0 | 0.00030667 | 2.5157e-05 |
| sim | right_pick_x35_y20_z18 | right | right_wrist_yaw_link | 3 | 0.011581 | 0.0095433 | 7.3333e-05 | 0.00656 | 0.00072467 |
| sim | right_pick_x35_y30_z06 | right | right_wrist_yaw_link | 3 | 0 | 0 | 0 | 0 | 3.6733e-06 |
| sim | right_pick_x35_y30_z10 | right | right_wrist_yaw_link | 3 | 0 | 0 | 0 | 0 | 3.6733e-06 |
| sim | right_pick_x35_y30_z14 | right | right_wrist_yaw_link | 3 | 0 | 0 | 0 | 0 | 3.6733e-06 |
| sim | right_pick_x35_y30_z18 | right | right_wrist_yaw_link | 3 | 0.0057626 | 0.0046533 | 0.0012267 | 0.00317 | 0.00034285 |