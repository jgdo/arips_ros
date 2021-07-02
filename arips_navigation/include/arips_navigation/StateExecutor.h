#pragma once

#include <type_traits>

#include <arips_navigation/DrivingState.h>

/**
 *
 * Example:
 * class MyState: public StateExecutor<MyState, DrivingStateProto> { ... };
 *
 * @tparam TParent Parent of this class
 * @tparam TStateClass DrivingState or a class derived from it, e.g. DrivingStateProto
 */
template <class TParent, class TStateClass> class StateExecutor : public TStateClass {
public:
    static_assert(std::is_convertible<TStateClass*, DrivingState*>::value,
                  "TParent needs to be a subclass of this StateExecuter class");

    // static_assert(std::is_convertible<TParent*, StateExecutor*>::value,
    //               "TParent needs to be a subclass of this StateExecuter class");

    typedef void (TParent::*StateFunction)();

    using DrivingStateProto::DrivingStateProto;

    bool isActive() final { return mCurrentStateFunc || mInnerState; }

    void runCycle() final {
        mStateCount++;

        if (mCurrentStateFunc) {
            (dynamic_cast<TParent*>(this)->*mCurrentStateFunc)();
        } else if (mInnerState != nullptr) {
            // DO NOT REMOVE the "else" above, otherwise runCycleImpl() could set an inner state
            // such that both mCurrentStateFunc and mInnerState would be executed in one cycle

            mInnerState->runCycle();
            if (!mInnerState->isActive()) {
                setState(mNextStateFunc);
            }
        }
    }

protected:
    void setState(StateFunction stateFunc) {
        mCurrentStateFunc = stateFunc;
        mInnerState = nullptr;
        mNextStateFunc = nullptr;
        mStateCount = -1;
    }

    void execState(DrivingState* state, StateFunction nextState) {
        mCurrentStateFunc = nullptr;
        mInnerState = state;
        mNextStateFunc = nextState;
        mStateCount = -1;
    }

    [[nodiscard]] bool isStateInit() const { return mStateCount == 0; }

    [[nodiscard]] bool isStateFunc(StateFunction func) const { return mCurrentStateFunc == func; }

private:
    // State function to execute in cycle, the state is inactive if this pointer and mInnerState is
    // null
    StateFunction mCurrentStateFunc = nullptr;

    // Alternative state to mCurrentStateFunc, at most one of both can be not null
    DrivingState* mInnerState = nullptr;

    // Execute after mInnerState has finished, if nullptr, this state has finished
    StateFunction mNextStateFunc = nullptr;

    int mStateCount = -1;
};
