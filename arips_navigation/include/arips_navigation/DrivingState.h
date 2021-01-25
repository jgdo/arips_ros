#pragma once

/**
 * Represents the execution of something, e.g. navigation or docking.
 */
class DrivingState{
public:
    inline virtual ~DrivingState() {}
    
    /**
     * @return true iff currently following a plan.
     */
    virtual bool isActive() = 0;

    /**
     * Run one control cycle given current plan and robot state.
     */
    virtual void runCycle() = 0;
};