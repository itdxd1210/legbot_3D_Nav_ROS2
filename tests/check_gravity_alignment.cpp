#include "gravity_alignment.hpp"
#include <cassert>
#include <iostream>
#include <limits>
int main() {
    const Eigen::Vector3d up=Eigen::Vector3d::UnitZ();
    for (double roll: {-0.35, 0., 0.2}) for (double pitch: {-0.7853981633974483, -0.3, -0.05236, 0., 0.05236, 0.4, 0.7853981633974483, 0.85}) {
        const Eigen::Matrix3d actual=(Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())*
                                     Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())).toRotationMatrix();
        const Eigen::Vector3d measured=actual.transpose()*up*9.81;
        const auto q=fastlio_init::gravityAlignedRotation(measured);
        assert((q*measured-9.81*up).norm()<1e-10);
        assert((q.toRotationMatrix().transpose()*q.toRotationMatrix()-Eigen::Matrix3d::Identity()).norm()<1e-10);
        // True horizontal plane tangents remain horizontal in the chosen level frame.
        for (const auto &t: {Eigen::Vector3d::UnitX().eval(), Eigen::Vector3d::UnitY().eval()})
            assert(std::abs((q*(actual.transpose()*t)).z())<1e-10);
    }
    assert(fastlio_init::gravityAlignedRotation(up).angularDistance(Eigen::Quaterniond::Identity())<1e-10);
    bool rejected=false;
    try {fastlio_init::gravityAlignedRotation(Eigen::Vector3d::Zero());}
    catch(const std::invalid_argument&){rejected=true;}
    assert(rejected);
    std::cout<<"PASS gravity rotation: roll/pitch cases, static acceleration cancellation, horizontal plane, invalid input\n";
}
