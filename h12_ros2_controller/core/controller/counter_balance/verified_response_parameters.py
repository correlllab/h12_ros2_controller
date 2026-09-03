from dataclasses import dataclass

import numpy as np

from h12_ros2_controller.core.controller.counter_balance.residual_response_model import (
    ContextualR5Model,
    ContextualU5Model,
    N5Model,
    N5ValidityThresholds,
)


RESPONSE_REPORT_SHA256 = (
    '63c33d45f507cf454fad0be2e387022b2bc630458a051a00322db0ffd6fc495a'
)
NOMINAL_REPORT_SHA256 = (
    'e8a513e24e2585452e0c2cb4bc3acff9795ffb9f9b47db4bce75295eee301ba6'
)


@dataclass(frozen=True)
class VerifiedH2Models:
    '''Store source-bound verified Iteration 5 response models'''

    u5: ContextualU5Model
    r5: ContextualR5Model
    n5: N5Model
    n5_error_bound: np.ndarray


@dataclass(frozen=True)
class VerifiedH3Models:
    '''Store source-bound verified Iteration 5B response models'''

    u5: ContextualU5Model
    u5_carryover: np.ndarray
    r5: ContextualR5Model
    n5: N5Model
    n5_error_bound: np.ndarray


def verified_h2_models():
    '''Construct the frozen verified response model package'''
    u5 = ContextualU5Model(
        coefficients=np.array([
            [0.9490105767532476, -0.06585368682329709,
             0.08698430024867744, 0.012443007273439851],
            [0.8909987680198288, -0.016584018554382556,
             -0.02298547652550779, 0.0022244144290229913],
            [0.2861639830748092, 0.08354869822410152,
             -0.042971995858742236, -0.04920749229610616],
            [1.2978231505485007, 0.1122764133192572,
             0.018456556265673735, 0.02759039752253802],
        ]),
        context_center=np.array([
            [-0.10001149357270979, -0.4950190172208616, -0.25],
            [0.024273742648204092, -0.0006133703993469186, -0.25],
            [-0.010382143354504651, -0.10431221975823218, -0.25],
            [-0.03383433400468238, 0.11046860773032047, -0.25],
        ]),
        context_scale=np.array([
            [0.27278513309667324, 0.8428735130569883,
             0.9682458365518543],
            [0.23395220314349807, 0.5372700835271338,
             0.9682458365518543],
            [0.1246125076059495, 0.7220221880402906,
             0.9682458365518543],
            [0.12120812377518285, 0.6272422092369133,
             0.9682458365518543],
        ]),
        weak_direction_mask=np.array([False, False, True, False]),
    )
    r5 = ContextualR5Model(
        coefficients=np.array([
            [
                [-0.07954834927496768, -0.007684395849842238,
                 0.008536394832861227, -0.008142924369587289,
                 0.009132206674341082, -0.004543652444646209],
                [0.05999105505700215, -0.05048977229206518,
                 0.008277179554518689, -0.0013440024466577567,
                 -0.003184823152282163, -0.012986346439591526],
            ],
            [
                [-0.115175410504175, -0.02479307021437901,
                 -0.014384974813569141, 0.008609711135813224,
                 0.015694594394199787, -0.040589552579136684],
                [-0.41925677490899677, 0.044547421360259874,
                 -0.009418272848878056, -0.006077269539819718,
                 0.023122971080121804, -0.012675620552413369],
            ],
        ]),
        context_center=np.array([
            -0.0026888287779091684,
            -0.018047975018793704,
            -0.003855944958353718,
            0.003903256621075433,
            -0.25,
        ]),
        context_scale=np.array([
            0.018557470007422874,
            0.018040683518193455,
            0.02485859090206848,
            0.02642552925086448,
            0.9682458365518543,
        ]),
    )
    n5 = N5Model(
        rate_trend_gain=np.array([
            0.088093250411762,
            0.29476515952213495,
        ]),
        validity=N5ValidityThresholds(
            max_abs_tilt=np.array([
                0.050457245906738384, 0.049233666281816794,
            ]),
            max_abs_rate=np.array([
                0.0897511059373853, 0.19426606718221048,
            ]),
            max_tilt_step=np.array([
                0.01395369846044266, 0.004071285387111936,
            ]),
            max_rate_step=np.array([
                0.07330680657015445, 0.11207226569070373,
            ]),
            max_integration_error=np.array([
                0.013595978101765788, 0.0020632343730452797,
            ]),
            min_dt=0.016,
            max_dt=0.025,
            sign_deadband=np.array([0.001, 0.001]),
        ),
        moving_momentum_gain=np.array([
            [0.15147405725217478, -0.001543402691294169],
            [-0.05973972236812019, -0.03386451940552381],
        ]),
        nominal_momentum_gain=np.array([
            [0.11525944314096898, 0.06663219507193895],
            [0.05953784340332627, -0.09564953330601016],
        ]),
    )
    error_bound = np.array([
        0.04502156405566961,
        0.016073697983417214,
    ])
    error_bound.setflags(write=False)
    return VerifiedH2Models(u5, r5, n5, error_bound)


def verified_h3_models():
    '''Construct the frozen verified 60 ms response model package'''
    h2 = verified_h2_models()
    r5 = ContextualR5Model(
        coefficients=np.array([
            [
                [-0.053634186902897424, 0.0047469361210610815,
                 -0.009761751734199162, -0.0010812686178624547,
                 0.008048820770958512, 0.005311446118591436],
                [0.05305327145596866, -0.03980556245658364,
                 0.010108822837507907, -0.0033344394368414487,
                 0.004878932312147209, 0.028157018454283617],
            ],
            [
                [-0.0805122900048564, -0.0036788020834746873,
                 -0.017895783695287045, 0.006340946149227041,
                 0.010622213530567958, -0.04245259879972554],
                [-0.287466073538806, 0.03138023831532889,
                 -0.017847330660745075, -0.006454927486889056,
                 -0.0058120617649878, -0.015876935916452718],
            ],
        ]),
        context_center=np.array([
            -0.0026888287779091684,
            -0.018047975018793704,
            -0.003855944958353718,
            0.003903256621075433,
            -0.25,
        ]),
        context_scale=np.array([
            0.018557470007422874,
            0.018040683518193455,
            0.02485859090206848,
            0.02642552925086448,
            0.9682458365518543,
        ]),
    )
    n5 = N5Model(
        rate_trend_gain=np.array([
            0.10719404684757934,
            0.17287439624900938,
        ]),
        validity=h2.n5.validity,
        moving_momentum_gain=np.array([
            [0.03666381236792148, 0.02931759233832672],
            [-0.020197169402921714, -0.03988962121373786],
        ]),
        nominal_momentum_gain=np.array([
            [0.08342456269984842, 0.018565714579158423],
            [0.05927090943883043, -0.061038158352030185],
        ]),
    )
    carryover = np.array([
        0.4603534444724387,
        0.5235796273972086,
        0.0,
        0.3409621141164678,
    ])
    error_bound = np.array([
        0.05173746707119781,
        0.023508335232659466,
    ])
    carryover.setflags(write=False)
    error_bound.setflags(write=False)
    return VerifiedH3Models(h2.u5, carryover, r5, n5, error_bound)
