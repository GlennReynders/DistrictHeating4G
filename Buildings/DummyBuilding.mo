within DistrictHeating4G.Buildings;
model DummyBuilding

  extends Interfaces.Baseclasses.Building(
    redeclare D1_annex60CE                                building,
    redeclare IDEAS.VentilationSystems.Ideal ventilationSystem,
    redeclare IDEAS.Occupants.Standards.ISO13790 occupant,
    DH=true,
    redeclare HeatingSystem.Radiator                        heatingSystem(
        DH=true))
    annotation (Placement(transformation(extent={{-14,-2},{6,18}})));
end DummyBuilding;
