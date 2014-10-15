within DistrictHeating4G.Buildings;
model DummyBuilding

  extends Interfaces.Baseclasses.Building(
    redeclare IDEAS.HeatingSystems.Examples.DummyBuilding building(nZones=1,
        nEmb=0),
    redeclare IDEAS.VentilationSystems.None ventilationSystem,
    redeclare IDEAS.Occupants.Standards.ISO13790 occupant,
    DH=true,
    redeclare HeatingSystem.Radiator                        heatingSystem(
        DH=true))
    annotation (Placement(transformation(extent={{-14,-2},{6,18}})));
end DummyBuilding;
