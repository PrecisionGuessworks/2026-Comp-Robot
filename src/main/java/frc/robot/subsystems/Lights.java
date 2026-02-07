package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Lights extends SubsystemBase{

    // private static final RGBWColor kGreen = new RGBWColor(0, 217, 0, 0);
    // private static final RGBWColor kWhite = new RGBWColor(Color.kWhite).scaleBrightness(0.5);
    // private static final RGBWColor kViolet = RGBWColor.fromHSV(Degrees.of(270), 0.9, 0.8);
    // private static final RGBWColor kRed = RGBWColor.fromHex("#D9000000").orElseThrow();
    private static final RGBWColor kRed = new RGBWColor(255, 0, 0, 0);
    private static final RGBWColor kBlue = new RGBWColor(0, 0, 255, 0);
    private static final RGBWColor kYellow = new RGBWColor(255, 255, 0, 0);

    private static final int kSlot0StartIdx = 0;
    private static final int kSlot0EndIdx = 7;

    private static final int kSlot1StartIdx = 8;
    private static final int kSlot1EndIdx = 37;


    private static final int kSlot2StartIdx = 38;
    private static final int kSlot2EndIdx = 67;

    private final CANdle m_candle = new CANdle(60, "rio");
    private static final int kUpdateFreqHz = 100;
    private int PastControl = -1;
    private int CurrentControl = -1;

    public Lights() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.LED.StripType = StripTypeValue.GRB;
        config.LED.BrightnessScalar = 0.75;
        config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        m_candle.getConfigurator().apply(config);
    }

    // @Override
    // public void periodic() {

    //  }
    // m_candle.setControl(
    //                     new ColorFlowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
    //                         .withColor(kViolet)
    //                 );



    public void setNotConnected() {
        m_candle.setControl(
            new SingleFadeAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                .withColor(kYellow)
                .withFrameRate(85)
                .withUpdateFreqHz(kUpdateFreqHz)

        );
        m_candle.setControl(
            new SingleFadeAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                .withColor(kYellow)
                .withFrameRate(85)
                .withUpdateFreqHz(kUpdateFreqHz)

        );
        m_candle.setControl(
            new SingleFadeAnimation(kSlot2StartIdx, kSlot2EndIdx).withSlot(2)
                .withColor(kYellow)
                .withFrameRate(85)
                .withUpdateFreqHz(kUpdateFreqHz)

        );
        CurrentControl = 1;
    }

    public void setClearAll() {
        m_candle.setControl(
            new EmptyAnimation(0)
        );
        m_candle.setControl(
            new EmptyAnimation(1)
        );
        m_candle.setControl(
            new EmptyAnimation(2)
        );
    }   

    public void setConnectedAlliance() {
        RGBWColor temp = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ?
            kBlue : kRed;


        m_candle.setControl(
            new LarsonAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                .withColor(temp)
                .withFrameRate(85)
                .withUpdateFreqHz(kUpdateFreqHz)
                .withSize(3)
                .withBounceMode(LarsonBounceValue.Center)

        );
        m_candle.setControl(
            new LarsonAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                .withColor(temp)
                .withFrameRate(85)
                .withUpdateFreqHz(kUpdateFreqHz)
                .withSize(6)
                .withBounceMode(LarsonBounceValue.Center)
        );
        m_candle.setControl(
            new LarsonAnimation(kSlot2StartIdx, kSlot2EndIdx).withSlot(2)
                .withColor(temp)
                .withFrameRate(85)
                .withUpdateFreqHz(kUpdateFreqHz)
                .withSize(6)
                .withBounceMode(LarsonBounceValue.Center)
        );
        CurrentControl = 2;
    }


    public void setParty(){
        m_candle.setControl(
            new RainbowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                .withFrameRate(85)
                .withUpdateFreqHz(kUpdateFreqHz)

        );
        m_candle.setControl(
            new RainbowAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                .withFrameRate(85)
                .withUpdateFreqHz(kUpdateFreqHz)
        );
        m_candle.setControl(
            new RainbowAnimation(kSlot2StartIdx, kSlot2EndIdx).withSlot(2)  
                .withFrameRate(85)
                .withUpdateFreqHz(kUpdateFreqHz)
        );
        CurrentControl = 3;
    }

    public void setFire(){
        double cooling = 0.5;
        double sparking = 0.5;
        m_candle.setControl(
            new FireAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                .withFrameRate(85)
                .withUpdateFreqHz(kUpdateFreqHz)
                .withCooling(cooling)
                .withDirection(AnimationDirectionValue.Forward)
                .withSparking(sparking)

        );
        m_candle.setControl(
            new FireAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                .withFrameRate(85)
                .withUpdateFreqHz(kUpdateFreqHz)
                .withCooling(cooling)
                .withDirection(AnimationDirectionValue.Forward)
                .withSparking(sparking)
        );
        m_candle.setControl(
            new FireAnimation(kSlot2StartIdx, kSlot2EndIdx).withSlot(2)  
                .withFrameRate(85)
                .withUpdateFreqHz(kUpdateFreqHz)
                .withCooling(cooling)
                .withDirection(AnimationDirectionValue.Forward)
                .withSparking(sparking)
        );
        CurrentControl = 4;
    }

    public void setAttack(){
        m_candle.setControl(
            new ColorFlowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                .withColor(kRed)
                .withFrameRate(100)
                .withUpdateFreqHz(kUpdateFreqHz)

        );
        m_candle.setControl(
            new ColorFlowAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                .withColor(kRed)
                .withFrameRate(100)
                .withUpdateFreqHz(kUpdateFreqHz)

        );
        m_candle.setControl(
            new ColorFlowAnimation(kSlot2StartIdx, kSlot2EndIdx).withSlot(2)
                .withColor(kRed)
                .withFrameRate(100)
                .withUpdateFreqHz(kUpdateFreqHz)

        );
        CurrentControl = 5;
    }


    public void UpdatedControl(int control){
        PastControl = CurrentControl;
        CurrentControl = control;
    }

    public void setPreviousControl(){
        CurrentControl = PastControl;
        if (CurrentControl == 1){
            setNotConnected();
        }
        else if (CurrentControl == 2){
            setConnectedAlliance();
        }
        else if (CurrentControl == 3){
            setParty();
        }
        else if (CurrentControl == 4){
            setFire();
        } else if (CurrentControl == 5){
            setAttack();
        } else {
            setClearAll();
        }

    }


}
