import sys
import asyncio
import base64
import cbor2
from coppeliasim_zmqremoteapi_client.asyncio import RemoteAPIClient

if sys.platform == 'win32' and sys.version_info >= (3, 8, 0):
    asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

async def main():
    async with RemoteAPIClient() as client:
        sim = await client.require('sim')
        print("✅ Connected to CoppeliaSim")
        await sim.startSimulation()

        try:
            while True:
                raw = await sim.getStringSignal('measuredDataCBOR')
                if raw:
                    decoded = base64.b64decode(raw)
                    points = cbor2.loads(decoded)
                    print(f"📡 Received {len(points)} points")
                    for pt in points[:5]:  # Preview first 5
                        x, y, z, dist = pt
                        print(f"  -> x={x:.2f}, y={y:.2f}, z={z:.2f}, distance={dist:.2f}")
                else:
                    print("⏳ Waiting for data...")
                await asyncio.sleep(0.5)
        except KeyboardInterrupt:
            print("🛑 Simulation interrupted by user.")
        finally:
            await sim.stopSimulation()

asyncio.run(main())
