#include "phase5_velocity.hpp"

#include <iostream>

int main(){
    const auto results=locomotion5::runSelfTests();
    int passed=0;
    for(const auto& r:results){
        std::cout<<(r.pass?"[PASS] ":"[FAIL] ")
                 <<r.name<<'\n';
        if(r.pass)++passed;
    }

    const auto reference=locomotion::makeReferenceBody();
    const auto envelope=
        locomotion5::evaluateVelocityEnvelope(
            reference,9,81
        );

    const auto slow=locomotion5::profileForSpeed(0.40);
    const auto cruise=locomotion5::profileForSpeed(0.60);
    const auto fast=locomotion5::profileForSpeed(0.80);

    std::cout<<"slow cadence = "
             <<slow.cadenceSpm<<" spm\n";
    std::cout<<"cruise cadence = "
             <<cruise.cadenceSpm<<" spm\n";
    std::cout<<"fast cadence = "
             <<fast.cadenceSpm<<" spm\n";

    std::cout<<"envelope valid = "<<envelope.valid
             <<" gait="<<envelope.allGaitsValid
             <<" reach="<<envelope.allReachable
             <<" legOuter="<<envelope.allOuterLimits
             <<" arms="<<envelope.armLengthsPreserved
             <<" corridor="<<envelope.handCorridorValid
             <<" elbow="<<envelope.elbowEnvelopeValid
             <<"\n";

    std::cout<<"hand corridor = "
             <<envelope.minHandLateralNorm
             <<" .. "<<envelope.maxHandLateralNorm
             <<" shoulder widths\n";
    std::cout<<"elbow flex = "
             <<envelope.minElbowFlexDeg
             <<" .. "<<envelope.maxElbowFlexDeg
             <<" deg\n";
    std::cout<<"max arm length error = "
             <<envelope.maxArmLengthError<<'\n';
    std::cout<<"max speed error = "
             <<envelope.maxSteadySpeedError<<'\n';
    std::cout<<"max coordinated foot error = "
             <<envelope.maxCoordinatedSupportFootError
             <<'\n';
    std::cout<<"min toe clearance = "
             <<envelope.minCoreToeClearance<<'\n';

    bool printedViolation=false;
    for(int si=0;si<=8&&!printedViolation;++si){
        const double speed=
            locomotion5::lerp(
                locomotion5::kMinWalkingSpeed,
                locomotion5::kMaxWalkingSpeed,
                (double)si/8.0
            );
        locomotion5::VelocityCommand cmd{};
        cmd.desiredSpeed=speed;
        cmd.stepCount=6;
        for(int pi=0;pi<=80;++pi){
            const double phase=(double)pi/80.0;
            const auto s=
                locomotion5::sampleCoordinated(
                    reference,cmd,phase
                );
            if(!s.allOuterLimits){
                std::cout<<"first coordinated outer violation"
                         <<" speed="<<speed
                         <<" sequencePhase="<<phase
                         <<" pelvisYaw="<<s.pelvisYawDeg
                         <<" pelvisRoll="<<s.pelvisRollDeg
                         <<" pelvisShift="<<s.pelvisLateralShift
                         <<" Lhip="<<s.pose.left.angles.hipFlexionDeg
                         <<" Lknee="<<s.pose.left.angles.kneeFlexionDeg
                         <<" Lankle="<<s.pose.left.angles.ankleDorsiflexionDeg
                         <<" Rhip="<<s.pose.right.angles.hipFlexionDeg
                         <<" Rknee="<<s.pose.right.angles.kneeFlexionDeg
                         <<" Rankle="<<s.pose.right.angles.ankleDorsiflexionDeg
                         <<"\n";
                printedViolation=true;
                break;
            }
        }
    }

    std::cout<<passed<<" / "
             <<results.size()
             <<" tests passed\n";
    return passed==(int)results.size()?0:1;
}
