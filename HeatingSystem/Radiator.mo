within DistrictHeating.HeatingSystem;
model Radiator
  extends Interfaces.PartialDistrictHeating(hex(
      m1_flow_nominal=1,
      m2_flow_nominal=1,
      m1_flow(start=1),
      dp1_nominal=1,
      dp2_nominal=1));

equation
  QHeaSys = -sum(emission.heatPortCon.Q_flow) - sum(emission.heatPortRad.Q_flow);
  P[1] = sum(pumpRad.PEl);
  Q[1] = 0;

end Radiator;
