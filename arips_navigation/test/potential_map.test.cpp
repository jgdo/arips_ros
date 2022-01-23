#include <gtest/gtest.h>

#include <arips_navigation/path_planning/PotentialMap.h>

PotentialMap toPotmap(const Eigen::MatrixXd& mat) {
    PotentialMap potmap(mat.cols(), mat.rows(), GridMapGeometry{"", {0, 0}, 1.0}, Pose2D{},
                        CostFunction{});

    for (int x = 0; x < mat.rows(); x++) {
        for (int y = 0; y < mat.cols(); y++) {
            potmap.mMatrix(x, y) = PotentialMap::Cell{mat(mat.cols()-y-1, x), true};
        }
    }

    return potmap;
}

TEST(PotentialMap, Gradien3x3) {
    Eigen::MatrixXd mat(3,3);

    mat << 3, 2, 1,
        3, 2, 1,
        3, 2, 1;

    /*
    mat << 1,1,1,
        2, 2, 2,
        3, 3, 3; */

    mat << 8.96292, 9.0062, 9.05348,
        9.0823, 9.12558, 9.1753,
        9.24328, 9.28657, 9.32894;


    const auto potmap = toPotmap(mat);
    const auto grad = potmap.getGradient({1, 1});
    ASSERT_TRUE(grad.has_value());
    EXPECT_NEAR(*grad, 1.88625, 0.001);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
