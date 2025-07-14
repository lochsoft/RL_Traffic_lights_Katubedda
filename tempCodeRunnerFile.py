bus_flows = ("ptlines2flows.py "
             "--net-file network/katubedda_junction_network_new.net.xml "
             "--ptlines-file additionals/bus_lines.xml "
             "--ptstops-file additionals/bus_stops.add.xml "
             "--begin 0 "
             "--end 3600 "
             "--stop-duration 20 "
             "--output-file demand/bus_demand.rou.xml")

subprocess.run(bus_flows, shell=True)