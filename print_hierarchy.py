import sys
import asyncio
from coppeliasim_zmqremoteapi_client.asyncio import RemoteAPIClient

# Required for Windows + Python 3.8+
if sys.platform == 'win32' and sys.version_info >= (3, 8, 0):
    asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

async def main():
    async with RemoteAPIClient() as client:
        sim = await client.require('sim')

        # Get handle of Omnirob0
        parent = await sim.getObject('/Omnirob0')
        print(f"Omnirob0 Handle: {parent}")

        # Get all children (including the parent itself)
        all_handles = await sim.getObjectsInTree(parent, sim.handle_all, 0)
        print(f"All related objects (including parent): {all_handles}")

        # Exclude the parent itself
        children = [h for h in all_handles if h != parent]
        print(f"Children of Omnirob0: {children}")

        # Print names of children
        for child in children:
            name = await sim.getObjectAlias(child, 1)  # 1 = search
            print(f" - {child}: {name}")

asyncio.run(main())