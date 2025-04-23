package pedroPathing;

public class Crush {
    private static Crush instance;
    private int InitialPositionLeftOuttakeSlider = 0;
    private boolean slidersInitialized = false; // Flag to track initialization

    private Crush() {} // Private constructor to enforce singleton pattern

    public static Crush getInstance() {
        if (instance == null) {
            instance = new Crush();
        }
        return instance;
    }

    public void setInitialPositions(int left) {
        this.InitialPositionLeftOuttakeSlider = left;
        this.slidersInitialized = true; // Mark as initialized
    }


    public int getLeft() {
        return InitialPositionLeftOuttakeSlider;
    }

    public boolean areSlidersInitialized() {
        return slidersInitialized;
    }
}
