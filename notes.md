Building traffic every 15 minutes and rebuilding the base map every 1-2 weeks is the standard, correct architecture for a production Valhalla deployment.

However, because of the error you just experienced (edgeid beyond bounds), you have to manage the relationship between these two cycles very carefully. Here is exactly what you need to consider:

# 1. The Golden Rule: Traffic depends on the Base Map
Valhalla traffic tiles do not store GPS coordinates or road names; they only store an array of speeds. The 1st speed belongs to Edge ID 0, the 2nd speed belongs to Edge ID 1, etc.

When your 15-minute job builds traffic.tar, it must use the exact same valhalla_tiles.tar that is currently running in production.
If your 15-minute job uses a slightly newer or older OSM dataset to build the traffic, the Edge IDs will shift, and you will get the exact crash you just debugged.
# 2. The 1-2 Week Base Map Update (The Danger Zone)
When the time comes to update your base map (valhalla_tiles.tar) every 1-2 weeks, you cannot just replace the base map and leave the old traffic running. You must do a synchronized release:

Build the new valhalla_tiles.tar (from new OSM data).
Pause your 15-minute traffic job temporarily.
Point your traffic job to the new valhalla_tiles.tar and generate a fresh traffic.tar.
Deploy both the new valhalla_tiles.tar and the new traffic.tar to the routing server at the exact same time.
Restart the Valhalla server.
Resume your 15-minute traffic job (now pointing at the new base map).
# 3. Zero-Downtime Deployments
For the 15-minute traffic updates: You do not need to restart Valhalla. Valhalla memory-maps traffic.tar. If you safely overwrite traffic.tar in the background, Valhalla will instantly start using the new speeds without dropping any routing requests.
For the 1-2 week base map updates: You cannot update valhalla_tiles.tar on the fly. You must restart the Valhalla process, which means you should ideally use a Blue/Green deployment (spin up a new Valhalla container with the new map + new traffic, route user requests to it, and then kill the old container).
# 4. Map-Matching your Raw Data
To build traffic every 15 minutes, you are likely taking raw GPS traces or speed data and map-matching them to Valhalla edges.

The map-matching process (e.g., using Valhalla's trace_attributes or Meili) must also be aware of the 1-2 week cycle.
If a car drove on a road yesterday, but your 1-2 week update just deleted or split that road in OSM, the old GPS data cannot be applied to the new map. Your traffic pipeline must clear out old speed caches when the base map changes.
