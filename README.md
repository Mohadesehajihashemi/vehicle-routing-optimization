# OptimeAI Vehicle Routing Optimization Task - My Submission

Submitted by: Mohadeseh Haji Hashemi  
Date: December 22, 2025  
Email: Mohadesehajihashemi@gmail.com  
Phone: 09021379018

Hi OptimeAI team,

Thanks for giving me this task – I really enjoyed working on it and learned a lot.

I made sure to cover everything in the PDF: building the solution, evaluating how good it is, visualizing the routes and regions, and explaining my approach.

The main goals were to create practical delivery routes that:
- Use as few vehicles as possible
- Keep each vehicle's service area separate from the others (minimal overlap)
- Reduce the total distance traveled

All while respecting vehicle capacity, time windows for deliveries, and service times at each customer.

## What I Did
I wrote a complete Python script (M1.py) and used Google OR-Tools because it's one of the best tools out there for routing problems with capacity and time constraints.

Step by step:
- Loaded the data from the two CSV files (the first row in customer_info is the depot)
- Calculated distances using the Haversine formula to get real geographic distances
- Added all the constraints: capacity, service time, and time windows
- To minimize vehicles, I added a high fixed cost (100,000) for each vehicle – this forced the solver to use only 6 vehicles
- Used Guided Local Search with a 60-second limit to improve the solution
- For service regions, I built a Convex Hull around each route's customers and calculated overlap – got 0% overlap in the end

## Results
- Vehicles used: 6 (the minimum possible with demand 78 and capacity 15)
- Total distance: 1080.54 km
- Average distance per vehicle: about 180 km
- Average customers per vehicle: 13
- Region overlap: 0% (fully separated areas)
- Computation time: around 60 seconds
- All deliveries made within time windows

## Visualization
I made two visuals to show the results clearly:
- route_map.html: An interactive map (using folium) with colored routes, semi-transparent service regions, customer popups with time windows, and the depot marked
- route_analysis.png: A static image showing the routes, regions, and a bar chart of the key metrics

Files included:
- route_map.html – open in a browser to explore
- route_analysis.png – quick overview
- optimized_routes.csv – full route details (open in Excel)

## Challenges I Faced
The hardest part was balancing all three goals – reducing vehicles can increase distance. The high fixed cost and Guided Local Search helped find a good balance. Also, calculating overlap for small routes needed some careful handling.

## Ideas for Future Improvements
- Use real road distances (OSRM or Google Maps API) instead of Haversine
- Add more real-life constraints like max driving hours per day or fuel costs
- For larger datasets, start with K-Means clustering to speed things up
- Build a simple dashboard with Streamlit so people can upload data and see results easily
- Add the ability to update routes during the day if orders change

## How to Run It

pip install pandas numpy ortools folium matplotlib scipy
python M1.py

(Make sure the CSV files are in the same folder)

Thanks again for the task – I had fun solving it. I'm happy to hop on a call to go through the code or answer any questions.

Best,  
Mohadeseh Haji Hashemi  
Mohadesehajihashemi@gmail.com  
09021379018
