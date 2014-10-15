within DistrictHeating4G.Toplevel;
model WithInterfaces
  extends DistrictHeating4G.Interfaces.Network(
    redeclare Buildings.DummyBuilding building1,
    redeclare Buildings.DummyBuilding building,
    redeclare Substations.HXWithBypass substation,
    redeclare Substations.HXWithBypass substation1,
    redeclare Production.BoilerWithPump production(boiler(QNom(displayUnit="kW")=
             10000, m_flow_nominal=0.2)),
    TAmb(k=273 - 10));

end WithInterfaces;
