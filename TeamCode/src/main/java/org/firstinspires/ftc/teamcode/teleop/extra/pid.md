public static PIDCoefficients pidCoeffsLF = new PIDCoefficients(0, 0, 0);
public PIDCoefficients pidGainsLF = new PIDCoefficients(0, 0, 0);

    public static PIDCoefficients pidCoeffsLB = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGainsLB = new PIDCoefficients(0, 0, 0);

    public static PIDCoefficients pidCoeffsRF = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGainsRF = new PIDCoefficients(0, 0, 0);

    public static PIDCoefficients pidCoeffsRB = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGainsRB = new PIDCoefficients(0, 0, 0);

    public static PIDCoefficients pidCoeffsCarousel = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGainsCarousel = new PIDCoefficients(0, 0, 0);

    public static PIDCoefficients pidCoeffsCascade = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGainsCascade = new PIDCoefficients(0, 0, 0);

    public static PIDCoefficients pidCoeffsIntake = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGainsIntake = new PIDCoefficients(0, 0, 0);


    private ElapsedTime PIDTimerLF = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime PIDTimerLB = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime PIDTimerRF = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime PIDTimerRB = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime PIDTimerCarousel = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime PIDTimerCascade = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime PIDTimerIntake = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

double integralLF = 0;
double lastErrorLF = 0;

    public void PIDLF(double targetVelocity) {
        PIDTimerLF.reset();
        double currentVelocity = left_front.getVelocity();

        double error = targetVelocity - currentVelocity;

        integralLF += error * PIDTimerLF.time();

        double deltaError = error - lastErrorLF;
        double derivative = deltaError / PIDTimerLF.time();

        pidGainsLF.p = pidCoeffsLF.p * error;
        pidGainsLF.i = pidCoeffsLF.i * integralLF;
        pidGainsLF.d = pidCoeffsLF.d = pidCoeffsLF.d * derivative;


        left_front.setVelocity(pidGainsLF.p + pidGainsLF.i + pidGainsLF.d + targetVelocity);

        lastErrorLF = error;
    }

    double integralLB = 0;
    double lastErrorLB = 0;

    public void PIDLB(double targetVelocity) {
        PIDTimerLB.reset();
        double currentVelocity = left_back.getVelocity();

        double error = targetVelocity - currentVelocity;

        integralLB += error * PIDTimerLB.time();

        double deltaError = error - lastErrorLB;
        double derivative = deltaError / PIDTimerLB.time();

        pidGainsLB.p = pidCoeffsLB.p * error;
        pidGainsLB.i = pidCoeffsLB.i * integralLB;
        pidGainsLB.d = pidCoeffsLB.d = pidCoeffsLB.d * derivative;


        left_back.setVelocity(pidGainsLB.p + pidGainsLB.i + pidGainsLB.d + targetVelocity);

        lastErrorLB = error;

    }

    double integralRF = 0;
    double lastErrorRF = 0;

    public void PIDRF(double targetVelocity) {
        PIDTimerRF.reset();
        double currentVelocity = right_front.getPower();

        double error = targetVelocity - currentVelocity;

        integralRF += error * PIDTimerRF.time();

        double deltaError = error - lastErrorRF;
        double derivative = deltaError / PIDTimerRF.time();

        pidGainsRF.p = pidCoeffsRF.p * error;
        pidGainsRF.i = pidCoeffsRF.i * integralRF;
        pidGainsRF.d = pidCoeffsRF.d = pidCoeffsRF.d * derivative;


        right_front.setVelocity(pidGainsRF.p + pidGainsRF.i + pidGainsRF.d + targetVelocity);

        lastErrorRF = error;
    }

    double integralRB = 0;
    double lastErrorRB = 0;

    public void PIDRB(double targetVelocity) {
        PIDTimerRB.reset();
        double currentVelocity = right_back.getVelocity();

        double error = targetVelocity - currentVelocity;

        integralRB += error * PIDTimerRB.time();

        double deltaError = error - lastErrorRB;
        double derivative = deltaError / PIDTimerRB.time();

        pidGainsRB.p = pidCoeffsRB.p * error;
        pidGainsRB.i = pidCoeffsRB.i * integralRB;
        pidGainsRB.d = pidCoeffsRB.d = pidCoeffsRB.d * derivative;

        right_back.setVelocity(pidGainsRB.p + pidGainsRB.i + pidGainsRB.d + targetVelocity);

        lastErrorRB = error;
    }

    double integralCarousel = 0;
    double lastErrorCarousel = 0;

    public void PIDcarousel(double targetVelocity) {
        PIDTimerCarousel.reset();
        double currentVelocity = carousel.getVelocity();

        double error = targetVelocity - currentVelocity;

        integralRB += error * PIDTimerCarousel.time();

        double deltaError = error - lastErrorRB;
        double derivative = deltaError / PIDTimerCarousel.time();

        pidGainsCarousel.p = pidCoeffsCarousel.p * error;
        pidGainsCarousel.i = pidCoeffsCarousel.i * integralRB;
        pidGainsCarousel.d = pidCoeffsCarousel.d = pidCoeffsCarousel.d * derivative;

        carousel.setVelocity(pidGainsCarousel.p + pidGainsCarousel.i + pidGainsCarousel.d + targetVelocity);

        lastErrorCarousel = error;
    }

    double integralCascade = 0;
    double lastErrorCascade = 0;

    public void PIDcascade(double targetVelocity) {
        PIDTimerCascade.reset();
        double currentVelocity = cascadingLift.getVelocity();

        double error = targetVelocity - currentVelocity;

        integralCascade += error * PIDTimerCascade.time();

        double deltaError = error - lastErrorCascade;
        double derivative = deltaError / PIDTimerCascade.time();

        pidGainsCascade.p = pidCoeffsCascade.p * error;
        pidGainsCascade.i = pidCoeffsCascade.i * integralCascade;
        pidGainsCascade.d = pidCoeffsCascade.d = pidCoeffsCascade.d * derivative;

        cascadingLift.setVelocity(pidGainsCascade.p + pidGainsCascade.i + pidGainsCascade.d + targetVelocity);

        lastErrorCascade = error;
    } 