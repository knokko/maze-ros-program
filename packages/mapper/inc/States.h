#pragma once

class Duckie;

class AbstractState {
    public:
        AbstractState(Duckie& duckie)
              : duckie(duckie) {
        }

        ~AbstractState() {
        }

        virtual void checkData()   = 0;
        virtual void plan()        = 0;
        virtual void driveAction() = 0;

    protected:
        void    setState(AbstractState* state) { duckie.currentState.reset(state); };

        Duckie& duckie;
};

class ExploringState : public AbstractState {
        void checkData() override;
        void plan() override;
        void driveAction() override;
};

class DrivingState : public AbstractState {
        void checkData() override;
        void plan() override;
        void driveAction() override;
};

class RacingState : public AbstractState {
        void checkData() override;
        void plan() override;
        void driveAction() override;
};

class DebugState : public AbstractState {
        void checkData() override;
        void plan() override;
        void driveAction() override;
};