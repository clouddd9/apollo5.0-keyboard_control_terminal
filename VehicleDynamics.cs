/**
 * Copyright (C) 2016, Jaguar Land Rover
 * This program is licensed under the terms and conditions of the
 * Mozilla Public License, version 2.0.  The full text of the
 * Mozilla Public License is at https://www.mozilla.org/MPL/2.0/
 * 
 * Copyright (c) 2019 LG Electronics, Inc.
 */

using UnityEngine;
using System.Collections.Generic;
using System.Linq;

[System.Serializable]//代表串行化，添加该标记的目的在于，使类成员可以在属性面板中显示
public class AxleInfo//AxleInfo:代表车轴信息的AxleInfo类
{
    public WheelCollider wheel;
    public GameObject wheelVisual;
    public bool motor;//马达
    public bool steering;
    public float brakeBias = 0.5f;
    //以上均在Dynamics/Axles中显示
    [System.NonSerialized]
    public WheelHit hitWheel;//contact information for the wheel, reported by WheelCollider.

    [System.NonSerialized]
    public bool isGroundedWheel = false;

}
public enum IgnitionStatus { Off, On }//点火装置状态

public class VehicleDynamics : MonoBehaviour
{
    public List<AxleInfo> axles;
    private int numberOfDrivingWheels;

    public Rigidbody RB {get; private set; }
    public Vector3 centerOfMass = new Vector3(0f, 0.35f, 0f);
    private Vector3 lastRBPosition;

    [Tooltip("torque at peak of torque curve")]//鼠标停留时说明
    public float maxMotorTorque = 450f;//最大马达扭矩

    [Tooltip("torque at max brake")]
    public float maxBrakeTorque = 3000f;

    [Tooltip("steering range is +-maxSteeringAngle")]
    public float maxSteeringAngle = 39.4f;

    [Tooltip("idle rpm")]
    public float minRPM = 800f;//发动机点火后的最小转速

    [Tooltip("max rpm")]
    public float maxRPM = 8299f;

    [Tooltip("gearbox ratios")]//齿轮传动比，即档位
    public float[] gearRatios = new float[] { 4.17f, 3.14f, 2.11f, 1.67f, 1.28f, 1f, 0.84f, 0.67f };
    public float finalDriveRatio = 2.56f;//主减速器的传动比

    [Tooltip("min time between gear changes")]
    public float shiftDelay = 0.7f;

    [Tooltip("time interpolated for gear shift")]
    public float shiftTime = 0.4f;

    [Tooltip("torque curve that gives torque at specific percentage of max RPM")]
    /*
    纵轴：当前扭矩与最大修正扭矩的比值
    横轴：当前转速与最大转速的比值
    */
    public AnimationCurve rpmCurve;
    [Tooltip("curves controlling whether to shift up at specific rpm, based on throttle position")]
    public AnimationCurve shiftUpCurve;
    [Tooltip("curves controlling whether to shift down at specific rpm, based on throttle position")]
    public AnimationCurve shiftDownCurve;

    [Tooltip("Air Drag Coefficient")]
    public float airDragCoeff = 1.0f;
    [Tooltip("Air Downforce Coefficient")]
    public float airDownForceCoeff = 2.0f;
    [Tooltip("Tire Drag Coefficient")]
    public float tireDragCoeff = 4.0f;

    [Tooltip("wheel collider damping rate")]
    public float wheelDamping = 1f;

    [Tooltip("autosteer helps the car maintain its heading")]
    [Range(0, 1)]
    public float autoSteerAmount = 0.338f;

    [Tooltip("traction control limits torque based on wheel slip - traction reduced by amount when slip exceeds the tractionControlSlipLimit")]
    [Range(0, 1)]
    public float tractionControlAmount = 0.675f;
    public float tractionControlSlipLimit = 0.8f;

    [Tooltip("how much to smooth out real RPM")]
    public float RPMSmoothness = 20f;

    // combined throttle and brake input
    public float AccellInput { get; private set; } = 0f;

    //steering input
    public float SteerInput { get; private set; } = 0f;

    //handbrake
    public bool HandBrake { get; set; } = false;

    public float CurrentRPM { get; private set; } = 0f;
    public float CurrentSpeed { get; private set; } = 0.0f;
    public float CurrentSpeedMeasured { get; private set; } = 0f;
    
    private float oldRotation = 0f;
    //调整后的最大转矩
    private float tractionControlAdjustedMaxTorque = 0f;
    public float currentTorque = 0f;

    private const float LOW_SPEED = 5f;
    private const float LARGE_FACING_ANGLE = 50f;

    public float CurrentGear { get; private set; } = 1;
    private int targetGear = 1;
    private int lastGear = 1;
    private bool shifting = false;
    private float lastShift = 0.0f;
    public bool Reverse { get; private set; } = false;

    private float traction = 0f;

    private float odometer = 0f;
    private float mileTicker = 0f;
    private float consumptionTime = 0f;
    private float consumptionDistance = 0f;
    // private float fuelCapacity = 60f;
    private float fuelLevel = 60f;

    private const float ZERO_K = 273.15f;
    public float AmbientTemperatureK { get; private set; } = 295.15f;
    public float EngineTemperatureK { get; private set; } = 295.15f;
    public bool CoolingMalfunction { get; private set; } = false;

    public IgnitionStatus IgnitionStatus { get; private set; } = IgnitionStatus.On;
    
    private VehicleController vehicleController;

      private void Awake()
    {
        RB = GetComponent<Rigidbody>();//访问其他脚本。获取刚体组件的属性并访问
        vehicleController = GetComponent<VehicleController>();
        RB.centerOfMass = centerOfMass;
    }

    void OnEnable()
    {
        RecalcDrivingWheels();//返回驱动轮个数
        
        /*调整最大转矩，初始最大转矩只是理论最大转矩的0.325倍
        fixedupdate()中会调用TractionControl()函数，
        而TractionControl()函数会调用AdjustTractionControlTorque(float forwardSlip)
        根据实际轮胎前滑率调整最大转矩。若小于阈值，则不断增加最大转矩，若大于阈值，则不断减少最大转矩
        */
        tractionControlAdjustedMaxTorque = maxMotorTorque - (tractionControlAmount * maxMotorTorque);

        axles[0].wheel.ConfigureVehicleSubsteps(5.0f, 30, 10);
        axles[1].wheel.ConfigureVehicleSubsteps(5.0f, 30, 10);
        axles[2].wheel.ConfigureVehicleSubsteps(5.0f, 30, 10);


        foreach (var axle in axles)
        {
            axle.wheel.wheelDampingRate = wheelDamping;
        }

        AmbientTemperatureK = ZERO_K + 22.0f; // a pleasant 22° celsius ambient
        EngineTemperatureK = ZERO_K + 22.0f;

        lastRBPosition = RB.position;
    }

    public void Update()
    {
        if (axles[0].wheel.wheelDampingRate != wheelDamping)
        {
            foreach (var axle in axles)
            {
                axle.wheel.wheelDampingRate = wheelDamping;
            }
        }

        //update wheel visual component rotations
        foreach (var axle in axles)
        {
            ApplyLocalPositionToVisuals(axle.wheel, axle.wheelVisual);
        }
    }

    public void FixedUpdate()
    {
        if (vehicleController != null)
        {
            SteerInput = vehicleController.SteerInput;
            AccellInput = vehicleController.AccelInput;
        }

        //air drag (quadratic)
        RB.AddForce(-airDragCoeff * RB.velocity * RB.velocity.magnitude);

        //downforce (quadratic)
        RB.AddForce(-airDownForceCoeff * RB.velocity.sqrMagnitude * transform.up);

        //tire drag (Linear)
        RB.AddForceAtPosition(-tireDragCoeff * RB.velocity, transform.position);

        //calc current gear ratio
        float gearRatio = Mathf.Lerp(gearRatios[Mathf.FloorToInt(CurrentGear) - 1], gearRatios[Mathf.CeilToInt(CurrentGear) - 1], CurrentGear - Mathf.Floor(CurrentGear));
        if (Reverse)
        {
            gearRatio = -1.0f * gearRatios[0];
        }

        //calc engine RPM from wheel rpm
        float wheelsRPM = (axles[1].wheel.rpm + axles[2].wheel.rpm) / 2f;
        if (wheelsRPM < 0)
            wheelsRPM = 0;

        // if the engine is on, the fuel injectors are going to be triggered at minRPM
        // to keep the engine running.  If the engine is OFF, then the engine will eventually
        // go all the way down to 0, because there's nothing keeping it spinning.
        var minPossibleRPM = IgnitionStatus == IgnitionStatus.On ? minRPM : 0.0f;
        CurrentRPM = Mathf.Lerp(CurrentRPM, minPossibleRPM + (wheelsRPM * finalDriveRatio * gearRatio), Time.fixedDeltaTime * RPMSmoothness);
        // I don't know why, but logging RPM while engine is off and we're not moving, is showing
        // a constant drift between 0.0185 and 0.0192 or so .. so just clamp it down to 0 at that
        // point.
        if (CurrentRPM < 0.02f)
        {
            CurrentRPM = 0.0f;
        }

        //find out which wheels are on the ground
        foreach (var axle in axles)
        {
            axle.isGroundedWheel = axle.wheel.GetGroundHit(out axle.hitWheel);
        }

        //convert inputs to torques
        float steer = maxSteeringAngle * SteerInput;
        currentTorque = (float.IsNaN(CurrentRPM / maxRPM)) ? 0.0f : rpmCurve.Evaluate(CurrentRPM / maxRPM) * gearRatio * finalDriveRatio * tractionControlAdjustedMaxTorque;

        foreach (var axle in axles)
        {
            if (axle.steering)
            {
                axle.wheel.steerAngle = steer;
            }
        }

        if (HandBrake)
        {
            //Make the accellInput negative so that brakes are applied in ApplyTorque()
            AccellInput = -1.0f;
        }

        // No autodrive while engine is off.
        if (IgnitionStatus == IgnitionStatus.On)
        {
            AutoSteer();
        }

        ApplyTorque();
        TractionControl();

        //shift if need be. No auto shifting while engine is off.
        if (IgnitionStatus == IgnitionStatus.On)
        {
            AutoGearBox();
        }

        //record current speed in MPH
        CurrentSpeed = RB.velocity.magnitude * 2.23693629f;

        float deltaDistance = wheelsRPM / 60.0f * (axles[1].wheel.radius * 2.0f * Mathf.PI) * Time.fixedDeltaTime;
        odometer += deltaDistance;
        mileTicker += deltaDistance;

        if ((mileTicker * 0.00062137f) > 1)
        {
            mileTicker = 0;
            SIM.LogSimulation(SIM.Simulation.MileTick);
        }

        /*
        // why does this not work :(
        float currentRPS = currentRPM / 60.0f;

        float accel = Mathf.Max(0.0f, accellInput);
        float angularV = currentRPS * Mathf.PI * 2.0f;

        float power = currentTorque //(rpmCurve.Evaluate(currentRPM / maxRPM) * accel * maxMotorTorque)  //Nm
            * angularV; // Watt

        float energy = power * Time.fixedDeltaTime  // w/s
            / 1000.0f * 3600.0f;                    // kw/h

        print("p:" + power + " e:" + energy);
        //approximation function for
        // https://en.wikipedia.org/wiki/Brake_specific_fuel_consumption#/media/File:Brake_specific_fuel_consumption.svg
        // range ~ 200-400 g/kWh
        float bsfc = (206.0f + Mathf.Sqrt(Mathf.Pow(currentRPM - 2200, 2.0f)
                            + Mathf.Pow((accel - 0.9f) * 10000.0f, 2.0f)) / 80 + currentRPM / 4500); // g/kWh

        float gasolineDensity = 1f / .75f;      // cm^3/g

        float deltaConsumption = bsfc * energy  // g
            * gasolineDensity                   // cm^3
            / 1000.0f;                          // l
        */

        // FIXME fix the more correct method above...
        float deltaConsumption = CurrentRPM * 0.00000001f
                               + currentTorque * 0.000000001f * Mathf.Max(0.0f, AccellInput);

        // if engine is not powered up, or there's zero acceleration, AND we're not idling
        // the engine to keep it on, then the fuel injectors are off, and no fuel is being used
        // idling == non-scientific calculation of "minRPM + 25%".
        if (IgnitionStatus != IgnitionStatus.On || (AccellInput <= 0.0f && CurrentRPM > minRPM + (minRPM * 0.25)))
        {
            deltaConsumption = 0.0f;
        }

        consumptionDistance = deltaConsumption / deltaDistance;     // l/m
        consumptionTime = deltaConsumption / Time.fixedDeltaTime;   // l/s

        fuelLevel -= deltaConsumption;

        float engineWeight = 200.0f; // kg
        float energyDensity = 34.2f * 1000.0f * 1000.0f; // J/l fuel
        float specificHeatIron = 448.0f; // J/kg for 1K temperature increase

        EngineTemperatureK += (deltaConsumption * energyDensity) / (specificHeatIron * engineWeight);

        float coolFactor = 0.00002f; //ambient exchange
        if (EngineTemperatureK > ZERO_K + 90.0f && !CoolingMalfunction)
            coolFactor += 0.00002f + 0.0001f * Mathf.Max(0.0f, CurrentSpeed); // working temperature reached, start cooling

        EngineTemperatureK = Mathf.Lerp(EngineTemperatureK, AmbientTemperatureK, coolFactor);

        //find current road surface type
        WheelHit hit;
        if (axles[0].wheel.GetGroundHit(out hit))
        {
            traction = hit.forwardSlip; // ground
            var roadObject = hit.collider.transform.parent == null ? hit.collider.transform : hit.collider.transform.parent;
        }
        else
        {
            traction = 0f; // air
        }

        CurrentSpeedMeasured = ((RB.position - lastRBPosition) / Time.fixedDeltaTime).magnitude;
        lastRBPosition = RB.position;
    }
    
    public void RecalcDrivingWheels()
    {
        //calculate how many wheels are driving
        numberOfDrivingWheels = axles.Where(a => a.motor).Count() * 2;
    }

    public void GearboxShiftUp()
    {
        if (Reverse)
        {
            Reverse = false;
            return;
        }
        lastGear = Mathf.RoundToInt(CurrentGear);
        targetGear = lastGear + 1;
        lastShift = Time.time;
        shifting = true;
    }

    public void GearboxShiftDown()
    {
        if (Mathf.RoundToInt(CurrentGear) == 1)
        {
            Reverse = true;
            return;
        }

        lastGear = Mathf.RoundToInt(CurrentGear);
        targetGear = lastGear - 1;
        lastShift = Time.time;
        shifting = true;
    }

    public void ShiftFirstGear()
    {
        if (Reverse == false)
            return;

        lastGear = 1;
        targetGear = 1;
        lastShift = Time.time;
        Reverse = false;
    }

    public void ShiftReverse()
    {
        lastGear = 1;
        targetGear = 1;
        lastShift = Time.time;
        Reverse = true;
    }

    public void ShiftReverseAutoGearBox()
    {
        if (Time.time - lastShift > shiftDelay)
        {
            if (CurrentRPM / maxRPM < shiftDownCurve.Evaluate(AccellInput) && Mathf.RoundToInt(CurrentGear) > 1)
            {
                GearboxShiftDown();
            }
        }

        if (CurrentGear == 1)
        {
            Reverse = true;
        }
    }

    private void AutoGearBox()
    {
        //check delay so we cant shift up/down too quick
        //FIXME lock gearbox for certain amount of time if user did override
        if (Time.time - lastShift > shiftDelay)
        {
            //shift up
            if (CurrentRPM / maxRPM > shiftUpCurve.Evaluate(AccellInput) && Mathf.RoundToInt(CurrentGear) < gearRatios.Length)
            {
                //don't shift up if we are just spinning in 1st
                if (Mathf.RoundToInt(CurrentGear) > 1 || CurrentSpeed > 15f)
                {
                    GearboxShiftUp();
                }
            }
            //else down
            if (CurrentRPM / maxRPM < shiftDownCurve.Evaluate(AccellInput) && Mathf.RoundToInt(CurrentGear) > 1)
            {
                GearboxShiftDown();
            }

        }

        if (shifting)
        {
            float lerpVal = (Time.time - lastShift) / shiftTime;
            CurrentGear = Mathf.Lerp(lastGear, targetGear, lerpVal);
            if (lerpVal >= 1f)
                shifting = false;
        }

        //clamp to gear range
        if (CurrentGear >= gearRatios.Length)
        {
            CurrentGear = gearRatios.Length - 1;
        }
        else if (CurrentGear < 1)
        {
            CurrentGear = 1;
        }
    }

    private void AutoSteer()
    {
        //bail if a wheel isn't on the ground
        foreach (var axle in axles)
        {
            if (axle.isGroundedWheel == false )
                return;
        }

        var yawRate = oldRotation - transform.eulerAngles.y;

        //don't adjust if the yaw rate is super high
        if (Mathf.Abs(yawRate) < 10f)
            RB.velocity = Quaternion.AngleAxis(yawRate * autoSteerAmount, Vector3.up) * RB.velocity;

        oldRotation = transform.eulerAngles.y;
    }

    private void ApplyTorque()
    {
        // acceleration is ignored when engine is not running, brakes are available still.
        if (AccellInput >= 0)
        {
            //motor
            float torquePerWheel = IgnitionStatus == IgnitionStatus.On ? AccellInput * (currentTorque / numberOfDrivingWheels) : 0f;
            foreach (var axle in axles)
            {
                if (axle.motor)
                {
                    axle.wheel.motorTorque = torquePerWheel;
                }

                axle.wheel.brakeTorque = 0f;
            }

        }
        // TODO: to get brake + accelerator working at the same time, modify this area.
        // You'll need to do some work to separate the brake and accel pedal inputs, though.
        // TODO: handBrake should apply full braking to rear axle (possibly all axles), without
        // changing the accelInput
        else
        {
            //brakes
            foreach (var axle in axles)
            {
                var brakeTorque = maxBrakeTorque * AccellInput * -1 * axle.brakeBias;
                axle.wheel.brakeTorque = brakeTorque;
                axle.wheel.motorTorque = 0f;
            }
        }
    }

    private void TractionControl()
    {
        foreach (var axle in axles)
        {
            if (axle.motor)
            {
                if (axle.wheel.isGrounded)
                    AdjustTractionControlTorque(axle.hitWheel.forwardSlip);
            }
        }
    }

    private void AdjustTractionControlTorque(float forwardSlip)
    {
        if (forwardSlip >= tractionControlSlipLimit && tractionControlAdjustedMaxTorque >= 0)
        {
            tractionControlAdjustedMaxTorque -= 10 * tractionControlAmount;
            if (tractionControlAdjustedMaxTorque < 0)
                tractionControlAdjustedMaxTorque = 0f;
        }
        else
        {
            tractionControlAdjustedMaxTorque += 10 * tractionControlAmount;
            if (tractionControlAdjustedMaxTorque > maxMotorTorque)
                tractionControlAdjustedMaxTorque = maxMotorTorque;
        }
    }

    public void ToggleIgnition()
    {
        switch (IgnitionStatus)
        {
            case IgnitionStatus.Off:
                StartEngine();
                break;
            case IgnitionStatus.On:
                StopEngine();
                break;
        }
    }

    public void StartEngine()
    {
        IgnitionStatus = IgnitionStatus.On;
    }

    public void StopEngine()
    {
        IgnitionStatus = IgnitionStatus.Off;
    }

    public void ToggleHandBrake()
    {
        HandBrake = !HandBrake;
    }

    public void SetHandBrake(bool enable)
    {
        HandBrake = enable;
    }

    public void ForceReset(Vector3 pos, Quaternion rot)
    {
        RB.MovePosition(pos);
        RB.MoveRotation(rot);
        RB.velocity = Vector3.zero;
        RB.angularVelocity = Vector3.zero;
        CurrentGear = 1;
        CurrentRPM = 0f;
        CurrentSpeed = 0f;
        currentTorque = 0f;
        AccellInput = 0f;
        SteerInput = 0f;
        foreach (var axle in axles)
        {
            axle.wheel.brakeTorque = Mathf.Infinity;
            axle.wheel.motorTorque = 0f;
        }
    }

    private void ApplyLocalPositionToVisuals(WheelCollider collider, GameObject visual)
    {
        if (visual == null) return;

        Vector3 position;
        Quaternion rotation;
        collider.GetWorldPose(out position, out rotation);

        visual.transform.position = position;
        visual.transform.rotation = rotation;
    }






}
