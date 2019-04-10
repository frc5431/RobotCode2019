package frc.robot.auto;

public enum Routine{
    ROCKET_TIER_1_RIGHT(Sequence.ROCKET_FORWARD_1, "hab_to_frocket_optimized_1", null, null, Sequence.ROCKET_FORWARD_1),
    ROCKET_TIER_1_LEFT(Sequence.ROCKET_FORWARD_1, "hab_to_frocket_optimized_1", null, null, Sequence.ROCKET_FORWARD_1, true),
    ROCKET_TIER_2_RIGHT(Sequence.ROCKET_FORWARD_2, "hab_to_frocket_optimized_1", null, null, Sequence.ROCKET_FORWARD_2),
    ROCKET_TIER_2_LEFT(Sequence.ROCKET_FORWARD_2, "hab_to_frocket_optimized_1", null, null, Sequence.ROCKET_FORWARD_1, true),
    ROCKET_TIER_3_RIGHT(Sequence.ROCKET_FORWARD_3, "hab_to_frocket_optimized_1", null, null, Sequence.ROCKET_FORWARD_1),
    ROCKET_TIER_3_LEFT(Sequence.ROCKET_FORWARD_3, "hab_to_frocket_optimized_1", null, null, Sequence.ROCKET_FORWARD_1, true),
    HAB_TO_FROCKET(null, "hab_to_frocket_gen", null, null),
    CARGOSHIP_LEFT_TO_RIGHT(Sequence.ROCKET_FORWARD_1, "hab_to_ccargo_gen", "ccargo_to_ls", "ls_to_ccargo_optimized_2"),
    CCARGO_TO_LS(Sequence.LOADING_STATION, "ccargo_to_ls", null, null),
    LS_TO_CCARGO(Sequence.ROCKET_FORWARD_1, "ls_to_ccargo", null, null),
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
