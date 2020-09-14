using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Extensions;
using Unity.Transforms;

namespace Planet
{
    public class PlanetSystem: SystemBase
    {
        protected override void OnUpdate()
        {
            var deltaTime = (float)Time.ElapsedTime;
            var otherPlanets = GetEntityQuery
                (
                    ComponentType.ReadOnly<PlanetTag>(),
                    ComponentType.ReadOnly<Translation>(),
                    ComponentType.ReadOnly<PhysicsMass>()
                )
                .ToEntityArray(Allocator.TempJob);
            
            Entities
                .WithAll<PlanetTag>()
                .ForEach((
                    Entity entity,
                    ref PhysicsVelocity velocity,
                    in Translation translation,
                    in PhysicsMass mass) =>
                {
                    for (var i = 0; i < otherPlanets.Length; i++)
                    {
                        var otherEntity = otherPlanets[i];
                        if (entity == otherEntity)
                        {
                            continue;
                        }

                        const double gravitationalConstant = 6.674E-11;

                        var otherTranslation = GetComponentDataFromEntity<Translation>(isReadOnly: true)[otherEntity];
                        var deltaTranslation = otherTranslation.Value - translation.Value;
                        var distanceSquared = (double) math.lengthsq(deltaTranslation);

                        var otherMass = GetComponentDataFromEntity<PhysicsMass>(isReadOnly: true)[otherEntity];

                        var force = (gravitationalConstant * MultiplyMasses(mass, otherMass)) / distanceSquared;
                        var direction = new double3(math.normalizesafe(deltaTranslation));
                        var forceVector = new float3(direction * force);

                        var impulse = forceVector * deltaTime;;

                        velocity.ApplyLinearImpulse(mass, impulse);
                    }

                })
                .WithDisposeOnCompletion(otherPlanets)
                .WithName(nameof(PlanetSystem))
                .ScheduleParallel();
        }
        
        private static float MultiplyMasses(in PhysicsMass firstMass, in PhysicsMass secondMass) 
            => math.pow(firstMass.InverseMass * secondMass.InverseMass, -1);
    }
}