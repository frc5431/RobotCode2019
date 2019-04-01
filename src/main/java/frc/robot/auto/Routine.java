package frc.robot.auto;

public enum Routine{
    CARGOSHIP_1_HATCH_RIGHT(Sequence.CARGO_SHIP, "hab_to_cargo", "cargo_to_ls", null),
    CARGOSHIP_1_HATCH_LEFT(Sequence.CARGO_SHIP, "hab_to_cargo", "cargo_to_ls", null, true),
    CARGOSHIP_2_HATCH_RIGHT(Sequence.CARGO_SHIP,"hab_to_cargo", "cargo_to_ls", "ls_to_cargo"),
    CARGOSHIP_2_HATCH_LEFT(Sequence.CARGO_SHIP,"hab_to_cargo", "cargo_to_ls", "ls_to_cargo", true),
    ROCKET_TIER_1_RIGHT(Sequence.ROCKET_FORWARD_1, "hab_to_frocket_optimized_1", "frocket_to_ls", "ls_to_crocket"),
    ROCKET_TIER_1_LEFT(Sequence.ROCKET_FORWARD_1, "hab_to_frocket_optimized_1", "frocket_to_ls", "ls_to_crocket", true),
    ROCKET_TIER_2_RIGHT(Sequence.ROCKET_FORWARD_2, "hab_to_frocket_optimized_1", "frocket_to_ls", "ls_to_crocket"),
    ROCKET_TIER_2_LEFT(Sequence.ROCKET_FORWARD_2, "hab_to_frocket_optimized_1", "frocket_to_ls", "ls_to_crocket", true),
    ROCKET_TIER_3_RIGHT(Sequence.ROCKET_FORWARD_3, "hab_to_frocket_optimized_1", "frocket_to_ls", "ls_to_crocket"),
    ROCKET_TIER_3_LEFT(Sequence.ROCKET_FORWARD_3, "hab_to_frocket_optimized_1", "frocket_to_ls", "ls_to_crocket", true),
    CARGOSHIP_1_ROCKET_FAR_1_RIGHT(Sequence.ROCKET_FORWARD_1, "hab_to_cargo", "cargo_to_ls", "ls_to_frocket"),
    CARGOSHIP_1_ROCKET_FAR_1_LEFT(Sequence.ROCKET_FORWARD_1, "hab_to_cargo", "cargo_to_ls", "ls_to_frocket", true),
    CARGOSHIP_1_ROCKET_CLOSE_1_RIGHT(Sequence.ROCKET_FORWARD_1, "hab_to_cargo", "cargo_to_ls", "ls_to_crocket"),
    CARGOSHIP_1_ROCKET_CLOSE_1_LEFT(Sequence.ROCKET_FORWARD_1, "hab_to_cargo", "cargo_to_ls", "ls_to_crocket", true),
    TEST(null, "TEST", null, null),
    TEST_OPTIMIZED(null, "TEST_optimized_1", null, null),
    DO_NOTHING(null, null, null, null);

    private final String startHatch, loadingStation, secondHatch;
    private final Sequence firstSequence, secondSequence;
    private final boolean swapped;

    Routine(final Sequence sequence, final String startHatch, final String loadingStation, final String secondHatch){
        this(sequence, startHatch, loadingStation, secondHatch, false);
    }

    Routine(final Sequence sequence, final String startHatch, final String loadingStation, final String secondHatch, final boolean swapped){
        this(sequence, startHatch, loadingStation, secondHatch, sequence, swapped);
    }

    Routine(final Sequence firstSequence, final String startHatch, final String loadingStation, final String secondHatch, final Sequence secondSequence){
        this(firstSequence, startHatch, loadingStation, secondHatch, secondSequence, false);
    }

    Routine(final Sequence firstSequence, final String startHatch, final String loadingStation, final String secondHatch, final Sequence secondSequence, final boolean swapped){
        this.firstSequence = firstSequence;
        this.secondSequence = secondSequence;
        this.startHatch = startHatch;
        this.loadingStation = loadingStation;
        this.secondHatch = secondHatch;
        this.swapped = swapped;
    }

    public Sequence getFirstSequence(){
        return firstSequence;
    }

    public Sequence getSecondSequence(){
        return secondSequence;
    }

    public String getStartHatchFile(){
        return startHatch;
    }

    public String getLoadingStationFile(){
        return loadingStation;
    }

    public String getSecondHatchFile(){
        return secondHatch;
    }

    public boolean isSwapped(){
        return swapped;
    }
};
