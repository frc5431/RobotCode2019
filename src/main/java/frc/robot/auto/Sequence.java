package frc.robot.auto;

public enum Sequence {
    FLOOR(ArmDirection.FORWARD),
    LOADING_STATION(ArmDirection.FORWARD),
    CARGO_SHIP(ArmDirection.FORWARD),
    ROCKET_FORWARD_1(ArmDirection.FORWARD),
    ROCKET_FORWARD_2(ArmDirection.FORWARD),
    ROCKET_FORWARD_3(ArmDirection.FORWARD),
    ROCKET_REVERSE_1(ArmDirection.REVERSE),
    ROCKET_REVERSE_2(ArmDirection.REVERSE),
    ROCKET_REVERSE_3(ArmDirection.REVERSE),
    STOW(ArmDirection.FORWARD),
    CLIMB(ArmDirection.REVERSE),
    INTAKE(ArmDirection.FORWARD),
    OUTTAKE(ArmDirection.FORWARD);

    private final ArmDirection direction;

    private Sequence(final ArmDirection dir){
        this.direction = dir;
    }

    public ArmDirection getDirection(){
        return direction;
    }
};