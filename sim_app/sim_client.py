# sim_app/sim_client.py
import sys
import asyncio
from coppeliasim_zmqremoteapi_client.asyncio import RemoteAPIClient

if sys.platform == 'win32' and sys.version_info >= (3, 8, 0):
    asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

async def get_sim():
    """
    Asynchronously creates and enters a RemoteAPIClient context, then retrieves the 'sim' resource.

    Returns:
        tuple: A tuple containing the RemoteAPIClient instance and the 'sim' resource.
    """
    client = await RemoteAPIClient().__aenter__()
    sim = await client.require('sim')
    return client, sim
