// Bring in my package's API, which is what I'm testing

#include <arips_arm_ik_plugin/IKComputations.h>

// Bring in gtest
#include <gtest/gtest.h>


using v3 = tf::Vector3;
using quat = tf::Quaternion;

template<class T>
bool near(T lhs, T rhs) {
    //using ::testing::internal::FloatingPoint;
    // return FloatingPoint<T>(lhs).AlmostEquals(FloatingPoint<T>(rhs));
    return std::abs(lhs-rhs) < 0.00001;
}

void compare(quat expected, quat actual) {
    bool eqpos = near(expected.x(), actual.x())
        && near(expected.y(), actual.y())
        && near(expected.z(), actual.z())
        && near(expected.w(), actual.w());

    bool eqneg = near(expected.x(), -actual.x())
                 && near(expected.y(), -actual.y())
                    && near(expected.z(), -actual.z())
                       && near(expected.w(), -actual.w());


    EXPECT_TRUE(eqpos || eqneg);
}

void compare(v3 expected, v3 actual) {
    EXPECT_FLOAT_EQ(expected.x(), actual.x());
    EXPECT_FLOAT_EQ(expected.y(), actual.y());
    EXPECT_FLOAT_EQ(expected.z(), actual.z());
}


void compare(std::vector<double> expected, std::vector<double> actual) {
    EXPECT_EQ(expected.size(), actual.size());

    for(size_t i = 0; i < expected.size(); i++) {
        EXPECT_NEAR(expected.at(i), actual.at(i), 0.00001);
    }
}

class CorrectionTest : public ::testing::TestWithParam<std::tuple<quat, quat, double, double>> {
};

TEST_P(CorrectionTest, TestCorrection)
{
    quat expected = std::get<0>(GetParam());
    quat orig = std::get<1>(GetParam());
    double baseAngle = std::get<2>(GetParam());
    double j5Expected = std::get<3>(GetParam());

    auto corrected = arips_arm_plugins::IKComputations::correctOrientation(orig, baseAngle);
    quat actual = corrected.first;
    double j5Actual = corrected.second;

    compare(expected, actual);
    EXPECT_NEAR(j5Actual, j5Expected, 0.0001);
}

INSTANTIATE_TEST_CASE_P(CorrectionTestInstance,
                        CorrectionTest,
                        ::testing::Values(
                                // forward => no correction
                                std::make_tuple(quat(0,0,0,1), quat(0,0,0,1), 0.0, 0.0),
                                // left => no correction
                                std::make_tuple(quat(v3(0,0,1),M_PI_2), quat(v3(0,0,1),M_PI_2), M_PI_2, 0.0),
                                // forward with rotated base => correction into base direction
                                std::make_tuple(quat(v3(0,0,1),0.3), quat(0,0,0,1), 0.3, 0.0),
                                // forward with rotated base in negative direction => correction to base direction
                                std::make_tuple(quat(v3(0,0,1),-0.3), quat(v3(0, 0, 1), 0.0), -0.3, 0.0),
                                // forward in rotated base direction => no correction
                                std::make_tuple(quat(v3(0,0,1),0.3), quat(v3(0, 0, 1), 0.3f), 0.3, 0.0),
                                // forward in other rotated base direction => no correction
                                std::make_tuple(quat(v3(0,0,1),-0.3), quat(v3(0, 0, 1), -0.3), -0.3, 0.0),
                                // overrotated in base direction => correct to base direction
                                std::make_tuple(quat(v3(0,0,1),0.3), quat(v3(0, 0, 1), 0.5), 0.3, 0.0),
                                // pointing down, no base rotation => no correction
                                std::make_tuple(quat(v3(0,1,0), M_PI_2), quat(v3(0, 1, 0), M_PI_2), 0.0, 0.0),
                                // pointing down, base rotation => no correction
                                std::make_tuple(quat(v3(0,1,0), M_PI_2), quat(v3(0, 1, 0), M_PI_2), 0.3, 0.0),
                                // pointing up, base rotation => no correction
                                std::make_tuple(quat(v3(0,1,0), -M_PI_2), quat(v3(0, 1, 0), -M_PI_2), 0.3, 0.0),
                                // forward tilted => no correction
                                std::make_tuple(quat(v3(1,0,0), 0.5), quat(v3(1,0,0), 0.5), 0.0, 0.5)
                        ));

struct IKTestParams {
    v3 pos;
    quat rot;
    std::vector<double> expsol;
};

class IKTest : public ::testing::TestWithParam<IKTestParams> {
};


static constexpr double len0 = 0.01, len1 = 0.07, len2 = 0.12, len3 = 0.15, len4 = 0.03, len5 = 0.02;

TEST_P(IKTest, theiktest) {
    std::vector<double> len = {len0, len1, len2, len3, len4, len5};
    std::vector<double> sol;

    IKTestParams params = GetParam();

    arips_arm_plugins::IKComputations::computeIK(params.pos, params.rot, len, sol);
    compare(params.expsol, sol);
}



INSTANTIATE_TEST_CASE_P(IKTestInstance,
                        IKTest,
                        ::testing::Values(
                                // up
//                                IKTestParams {v3(0, 0, len0 + len1 + len2+len3+len4+len5), quat(v3(0,1,0), -M_PI_2), {0,0,0,0,0} },
//                                // up, tip to front
//                                IKTestParams {v3(len4+len5, 0, len0 + len1 + len2+len3), quat(v3(0,1,0), 0), {0,0,0,M_PI_2,0}},
//                                // up, tip and l4 to front
//                                IKTestParams{v3(len3+len4+len5, 0, len0 + len1 + len2), quat(v3(0,1,0), 0), {0,0,M_PI_2,0, 0}},
//                                // forward
//                                IKTestParams{v3(len2+len3+len4+len5, 0, len0 + len1), quat(v3(0,1,0), 0), {0,M_PI_2, 0,0, 0}},
//                                // up rotated
//                                // TODO j5 rotation IKTestParams {v3(0, 0, len0 + len1 + len2+len3+len4+len5), quat(v3(0,0,1), -0.3) * quat(v3(0,1,0), -M_PI_2), {-0.3,0,0,0,0} }
//                                // tip front up
//                                //  |_
//                                //    |
//                                //    |
//                                IKTestParams {v3(len3, 0, len0 + len1 + len2 + len4 + len5), quat(v3(0,1,0), -M_PI_2), {0,0,M_PI_2,-M_PI_2,0}},
//                                // tip front up, skewed, requires rot correction
//                                //  |_               /
//                                //    |  fron side,  | from front
//                                //    |              |
//                                IKTestParams {v3(len3, 0, len0 + len1 + len2 + len4 + len5), quat(v3(1,0,0), -0.3)*quat(v3(0,1,0), -M_PI_2), {0,0,M_PI_2,-M_PI_2,0}},
//                                //  IKTestParams {v3(len3, 0, len0 + len1 + len2 + len4 + len5), quat(v3(1,0,0), -1.8)*quat(v3(0,1,0), -M_PI_2), {0,0,M_PI_2,-M_PI_2,0}}
//                                // up, tip to left
//                                IKTestParams {v3(0, len4+len5, len0 + len1 + len2+len3), quat(v3(0,0,1), M_PI_2), {M_PI_2,0,0,M_PI_2,0}},
                                IKTestParams {v3(0.223863182215, 0.05796219564, 0.1625287302), quat(0.01663151, 0.51452351175, 0.060132427, 0.8552035), {M_PI_2,0,0,M_PI_2,0}}
                        ));



// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
