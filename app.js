// Use configuration from config.js
const GOOGLE_MAPS_API_KEY = config.GOOGLE_MAPS_API_KEY;

let map, geocoder, directionsService, directionsRenderer, elevationService, distanceMatrixService;
let routesData = [];
let fullDirectionsResult = null;
let currentUser = null;

// Constants for CO₂ calculations
const EMISSION_FACTORS = {
  DRIVING: {
    SMALL_CAR: 0.15,    // kg CO₂ per km
    MEDIUM_CAR: 0.21,   // kg CO₂ per km
    LARGE_CAR: 0.30,    // kg CO₂ per km
    SUV: 0.35          // kg CO₂ per km
  },
  TRANSIT: 0.05,        // kg CO₂ per km per person (average for public transport)
  WALKING: 0,           // kg CO₂ per km
  BICYCLING: 0.02,      // kg CO₂ per km (including manufacturing/maintenance)
  // Detailed public transport factors
  PUBLIC_TRANSPORT: {
    BUS: 0.08,         // kg CO₂ per km per person
    TRAIN: 0.04,       // kg CO₂ per km per person
    SUBWAY: 0.03,      // kg CO₂ per km per person
    TRAM: 0.03         // kg CO₂ per km per person
  }
};

const TRAFFIC_MULTIPLIERS = {
  'none': 1.0,         // No traffic
  'light': 1.1,        // Light traffic
  'medium': 1.3,       // Medium traffic
  'heavy': 1.6         // Heavy traffic
};

const TERRAIN_MULTIPLIERS = {
  'flat': 1.0,         // Flat terrain
  'slight': 1.1,       // Slight incline
  'moderate': 1.25,    // Moderate incline
  'steep': 1.4         // Steep incline
};

const WEATHER_MULTIPLIERS = {
  'ideal': 1.0,        // Ideal conditions
  'rain': 1.1,         // Rain
  'snow': 1.2,         // Snow
  'strong_wind': 1.15  // Strong wind
};

// Climatiq API configuration
const CLIMATIQ_API_KEY = 'YOUR_CLIMATIQ_API_KEY'; // You'll need to get an API key from climatiq.io
const CLIMATIQ_BASE_URL = 'https://beta3.api.climatiq.io';

// Constants for CO₂ calculations based on road type
const ROAD_TYPE_FACTORS = {
  'highway': 1.0,      // Base factor for highways
  'arterial': 1.2,     // More stops and varying speeds
  'residential': 1.3,  // Frequent stops and lower speeds
  'rural': 1.1        // Less traffic but varying conditions
};

// Vehicle efficiency factors based on speed (km/h)
const SPEED_EFFICIENCY = {
  'slow': {            // 0-30 km/h
    factor: 1.4,       // Less efficient due to frequent stops/starts
    range: [0, 30]
  },
  'optimal': {         // 30-80 km/h
    factor: 1.0,       // Most efficient speed range
    range: [30, 80]
  },
  'high': {            // 80+ km/h
    factor: 1.2,       // Less efficient due to air resistance
    range: [80, Infinity]
  }
};

// Add speed limits for different road types (in km/h)
const SPEED_LIMITS = {
  'highway': {
    normal: 100,
    congested: 40
  },
  'major_road': {
    normal: 80,
    congested: 30
  },
  'secondary_road': {
    normal: 50,
    congested: 20
  },
  'residential': {
    normal: 30,
    congested: 15
  },
  'service_road': {
    normal: 30,
    congested: 15
  },
  'toll_road': {
    normal: 90,
    congested: 35
  },
  'unknown': {
    normal: 50,
    congested: 20
  }
};

// Add cycling speeds based on terrain (in km/h)
const CYCLING_SPEEDS = {
  'flat': 20,
  'slight_uphill': 15,
  'moderate_uphill': 10,
  'steep_uphill': 8,
  'slight_downhill': 25,
  'moderate_downhill': 30,
  'steep_downhill': 35
};

// Check if user is already logged in
window.onload = function() {
  const savedUser = localStorage.getItem('currentUser');
  if (savedUser) {
    currentUser = JSON.parse(savedUser);
    showUserProfile();
  } else {
    showLoginModal();
  }
};

function showLoginModal() {
  document.getElementById('loginModal').style.display = 'flex';
  document.getElementById('userProfile').style.display = 'none';
}

function showUserProfile() {
  document.getElementById('loginModal').style.display = 'none';
  document.getElementById('userProfile').style.display = 'block';
  updateUserStats();
}

function login() {
  const email = document.getElementById('email').value;
  const password = document.getElementById('password').value;

  if (!email || !password) {
    alert('Please enter both email and password');
    return;
  }

  // Get existing users or create empty array
  const users = JSON.parse(localStorage.getItem('users') || '[]');
  const user = users.find(u => u.email === email && u.password === password);

  if (user) {
    currentUser = user;
    localStorage.setItem('currentUser', JSON.stringify(user));
    showUserProfile();
    updateUserStats();
  } else {
    alert('Invalid email or password');
  }
}

function register() {
  const email = document.getElementById('email').value;
  const password = document.getElementById('password').value;

  if (!email || !password) {
    alert('Please enter both email and password');
    return;
  }

  // Get existing users or create empty array
  const users = JSON.parse(localStorage.getItem('users') || '[]');
  
  // Check if user already exists
  if (users.some(u => u.email === email)) {
    alert('User already exists');
    return;
  }

  // Create new user
  const newUser = {
    email,
    password,
    trips: [],
    totalCO2Saved: 0,
    totalTrips: 0
  };

  // Add to users array
  users.push(newUser);
  localStorage.setItem('users', JSON.stringify(users));

  // Log in the new user
  currentUser = newUser;
  localStorage.setItem('currentUser', JSON.stringify(newUser));
  showUserProfile();
}

function logout() {
  currentUser = null;
  localStorage.removeItem('currentUser');
  showLoginModal();
}

function updateUserStats() {
  if (!currentUser) return;

  document.getElementById('userName').textContent = currentUser.email;
  document.getElementById('totalCO2Saved').textContent = `${currentUser.totalCO2Saved.toFixed(2)} kg`;
  document.getElementById('totalTrips').textContent = currentUser.totalTrips;
}

function initMap() {
  try {
    console.log('Initializing map...');
    
    // Create the map instance
  map = new google.maps.Map(document.getElementById('map'), {
      center: { lat: 12.9716, lng: 77.5946 },
    zoom: 13,
      mapTypeControl: true,
      fullscreenControl: true,
      // Add 3D control options
      tilt: 0,
      mapTypeId: 'roadmap',
      mapTypeControlOptions: {
        mapTypeIds: ['roadmap', 'satellite']
      }
    });
    console.log('Map initialized successfully');

    // Initialize services
  geocoder = new google.maps.Geocoder();
  directionsService = new google.maps.DirectionsService();
  directionsRenderer = new google.maps.DirectionsRenderer({ map: map });
    elevationService = new google.maps.ElevationService();
    distanceMatrixService = new google.maps.DistanceMatrixService();

    // Initialize autocomplete
  initAutocomplete('start');
  initAutocomplete('end');
    
    console.log('All services initialized');

    // Add vehicle type selector for driving mode
    const modeSelect = document.getElementById('mode');
    modeSelect.addEventListener('change', function() {
      const vehicleTypeContainer = document.getElementById('vehicle-type-container');
      if (this.value === 'DRIVING') {
        vehicleTypeContainer.style.display = 'block';
      } else {
        vehicleTypeContainer.style.display = 'none';
      }
    });
  } catch (error) {
    console.error('Error initializing map:', error);
    document.getElementById('map').innerHTML = 'Error loading map. Please check console for details.';
  }
}

function initAutocomplete(inputId) {
  const input = document.getElementById(inputId);
  const autocomplete = new google.maps.places.Autocomplete(input);
  autocomplete.setFields(['place_id', 'geometry', 'name']);
}

// Add error handling for Google Maps API loading
window.gm_authFailure = function() {
  console.error('Google Maps authentication failed! Please check your API key.');
  document.getElementById('map').innerHTML = 'Error: Google Maps failed to load. Please check your API key.';
};

// Track commute function to calculate CO₂ emissions
async function trackCommute() {
  try {
    console.log('Starting trackCommute function...');
    
  const start = document.getElementById('start').value;
  const end = document.getElementById('end').value;
  const mode = document.getElementById('mode').value;

    if (!start || !end) {
      alert('Please enter both start and end locations');
      return;
    }

    console.log('Calculating route for:', {
      start: start,
      end: end,
      mode: mode
    });

    // Convert PUBLIC_TRANSPORT to TRANSIT for Google Maps API
    const travelMode = mode === 'PUBLIC_TRANSPORT' ? 'TRANSIT' : mode;

    const request = {
    origin: start,
    destination: end,
      travelMode: google.maps.TravelMode[travelMode],
      provideRouteAlternatives: true,
      transitOptions: travelMode === 'TRANSIT' ? {
        departureTime: new Date(),
        modes: ['BUS', 'RAIL', 'SUBWAY', 'TRAIN', 'TRAM'],
        routingPreference: 'FEWER_TRANSFERS'
      } : undefined,
      drivingOptions: travelMode === 'DRIVING' ? {
        departureTime: new Date(),
        trafficModel: google.maps.TrafficModel.BEST_GUESS
      } : undefined
    };

    // Add specific options for cycling
    if (mode === 'BICYCLING') {
      request.avoidHighways = true;
      request.avoidTolls = true;
      // Try to prefer cycling-friendly routes
      request.optimizeWaypoints = true;
    }

    console.log('Sending route request:', request);

    directionsService.route(request, async (result, status) => {
    if (status === 'OK') {
        console.log('Route calculated successfully');
      fullDirectionsResult = result;

        directionsRenderer.setDirections(result);
        
        try {
          routesData = await Promise.all(result.routes.map(async (route, index) => {
            console.log('Processing route', index + 1);
            const distanceVal = route.legs[0].distance.value / 1000;
            const emissionDetails = await calculateDetailedEmissions(route, mode);
            
            return {
              index,
              distanceText: route.legs[0].distance.text,
              distanceVal,
              ...emissionDetails
            };
          }));

          console.log('Routes processed:', routesData);
          displayRouteChoices(routesData);
        } catch (error) {
          console.error('Error processing routes:', error);
          alert('Error calculating emissions. Please try again.');
        }
      } else {
        console.error('Directions request failed:', status);
        // If cycling route fails, try alternative routing
        if (mode === 'BICYCLING') {
          console.log('Trying alternative cycling route...');
          // Try with WALKING mode but adjust the display
          const walkingRequest = {
            ...request,
            travelMode: google.maps.TravelMode.WALKING
          };
          directionsService.route(walkingRequest, async (walkResult, walkStatus) => {
            if (walkStatus === 'OK') {
              console.log('Alternative cycling route found');
              fullDirectionsResult = walkResult;
              directionsRenderer.setDirections(walkResult);
              
              try {
                routesData = await Promise.all(walkResult.routes.map(async (route, index) => {
                  const distanceVal = route.legs[0].distance.value / 1000;
                  const emissionDetails = await calculateDetailedEmissions(route, 'BICYCLING');

        return {
          index,
          distanceText: route.legs[0].distance.text,
          distanceVal,
                    ...emissionDetails,
                    isCyclingAlternative: true
        };
                }));

      displayRouteChoices(routesData);
              } catch (error) {
                console.error('Error processing alternative cycling routes:', error);
                alert('Error calculating cycling routes. Please try again.');
              }
            } else {
              alert('Could not find any suitable cycling routes. Please try different locations.');
            }
          });
    } else {
          alert('Could not calculate route. Please check your locations and try again.');
        }
    }
  });
  } catch (error) {
    console.error('Error in trackCommute:', error);
    alert('An error occurred while tracking your commute. Please try again.');
  }
}

function displayRouteChoices(routes) {
  console.log('Displaying route choices:', routes);
  const container = document.getElementById('route-options');
  
  // Show the container
  container.style.display = 'block';
  
  let html = `<h2>Available Routes</h2>`;

  routes.forEach((r, i) => {
    const details = r.details;
    const speedProfile = details.speedProfile;
    
    let speedBreakdown = '';
    if (speedProfile && speedProfile.speedsByRoadType) {
      speedBreakdown = Array.from(speedProfile.speedsByRoadType.entries())
        .map(([type, speed]) => {
          const readableType = type.split('_').map(word => 
            word.charAt(0).toUpperCase() + word.slice(1)
          ).join(' ');
          return `${readableType}: ${speed.toFixed(1)} km/h`;
        }).join('<br>');
    }

    let cyclingInfo = '';
    if (details.cyclingProfile) {
      const profile = details.cyclingProfile;
      cyclingInfo = `
        <li>Cycling Details:<br>
          Average Speed: ${profile.averageSpeed.toFixed(1)} km/h<br>
          Terrain Breakdown:<br>
          ${profile.segments.map(seg => 
            `- ${seg.type.replace('_', ' ')}: ${(seg.distance * 100 / r.distanceVal).toFixed(1)}% of route`
          ).join('<br>')}
        </li>
      `;
    }

    html += `
      <div class="route-option" id="route-${i}">
      <label>
        <input type="radio" name="route" value="${i}" onchange="selectRoute(${i})">
          <div class="route-details">
            <h3>Route ${i + 1}: ${r.distanceText}</h3>
            <p class="emission">CO₂ Emitted: ${r.totalEmission.toFixed(2)} kg</p>
            <div class="route-analysis">
              <p>Route Analysis:</p>
              <ul>
                <li>Distance: ${r.distanceText}</li>
                <li>Average Speed: ${speedProfile.averageSpeed.toFixed(1)} km/h</li>
                <li>Speed by Road Type:<br>${speedBreakdown}</li>
                ${cyclingInfo}
                ${details.averageTrafficDelay ? 
                  `<li>Traffic Delay: ${details.averageTrafficDelay.toFixed(1)}%</li>` : ''}
                ${details.totalElevationGain ? 
                  `<li>Elevation Changes:<br>
                    ↑ Gain: ${details.totalElevationGain.toFixed(0)}m<br>
                    ↓ Loss: ${details.totalElevationLoss.toFixed(0)}m</li>` : ''}
              </ul>
            </div>
          </div>
        </label>
      </div>
    `;
  });

  // Add start commute button
  html += `
    <button id="start-commute-btn" class="start-commute-btn" onclick="startCommute()" disabled>
      Start Commute
    </button>
  `;

  // Set the HTML content
  container.innerHTML = html;
}

function selectRoute(index) {
  console.log('Selected route:', index);
  
  // Update UI to show selected route
  document.querySelectorAll('.route-details').forEach(el => {
    el.classList.remove('selected-route');
  });
  
  const selectedRouteEl = document.querySelector(`#route-${index} .route-details`);
  if (selectedRouteEl) {
    selectedRouteEl.classList.add('selected-route');
  }
  
  // Enable start commute button
  const startButton = document.getElementById('start-commute-btn');
  if (startButton) {
    startButton.disabled = false;
  }
  
  // Update the map to show only the selected route
  if (fullDirectionsResult && fullDirectionsResult.routes[index]) {
    // Create a new DirectionsResult object with only the selected route
    const selectedRouteResult = {
      routes: [fullDirectionsResult.routes[index]],
      request: fullDirectionsResult.request,
      status: fullDirectionsResult.status
    };
    
    // Clear existing routes
    directionsRenderer.setMap(null);
    
    // Create a new renderer for the selected route
    directionsRenderer = new google.maps.DirectionsRenderer({
      map: map,
      directions: selectedRouteResult,
      routeIndex: 0,
      polylineOptions: {
        strokeColor: '#4CAF50',
        strokeWeight: 6,
        strokeOpacity: 0.8
      },
      suppressMarkers: false,
      preserveViewport: false
    });
    
    // Fit the map to the selected route bounds
    const bounds = new google.maps.LatLngBounds();
    const route = selectedRouteResult.routes[0];
    route.legs.forEach(leg => {
      leg.steps.forEach(step => {
        step.path.forEach(point => {
          bounds.extend(point);
        });
      });
    });
    map.fitBounds(bounds);
  }
}

function startCommute() {
  if (!currentUser) {
    alert('Please login to track your commute');
    showLoginModal();
    return;
  }

  const selectedRoute = document.querySelector('input[name="route"]:checked');
  if (!selectedRoute) {
    alert('Please select a route first');
    return;
  }

  const routeIndex = parseInt(selectedRoute.value);
  const route = routesData[routeIndex];
  
  // Get the start and end locations
  const start = document.getElementById('start').value;
  const end = document.getElementById('end').value;
  const mode = document.getElementById('mode').value;

  // Convert our travel mode to Google Maps URL travel mode
  const travelModeMap = {
    'DRIVING': 'driving',
    'WALKING': 'walking',
    'BICYCLING': 'bicycling',
    'PUBLIC_TRANSPORT': 'transit'
  };

  // Calculate CO2 saved (compared to driving)
  let co2Saved = 0;
  if (mode !== 'DRIVING') {
    // Get what would have been emitted if driving
    const drivingEmission = route.distanceVal * EMISSION_FACTORS.DRIVING.MEDIUM_CAR;
    co2Saved = drivingEmission - route.totalEmission;
  }

  // Save trip data
  const tripData = {
    date: new Date().toISOString(),
    start,
    end,
    mode,
    distance: route.distanceVal,
    co2Emission: route.totalEmission,
    co2Saved: co2Saved
  };

  // Update user data
  const users = JSON.parse(localStorage.getItem('users'));
  const userIndex = users.findIndex(u => u.email === currentUser.email);
  
  if (userIndex !== -1) {
    users[userIndex].trips.push(tripData);
    users[userIndex].totalCO2Saved += co2Saved;
    users[userIndex].totalTrips += 1;
    
    // Update localStorage
    localStorage.setItem('users', JSON.stringify(users));
    
    // Update current user
    currentUser = users[userIndex];
    localStorage.setItem('currentUser', JSON.stringify(currentUser));
    
    // Update UI
    updateUserStats();
  }

  // Construct Google Maps URL
  let googleMapsUrl = `https://www.google.com/maps/dir/?api=1` +
    `&origin=${encodeURIComponent(start)}` +
    `&destination=${encodeURIComponent(end)}` +
    `&travelmode=${travelModeMap[mode]}`;

  // Add waypoints if they exist
  const selectedGoogleRoute = fullDirectionsResult.routes[routeIndex];
  let waypoints = [];
  if (selectedGoogleRoute && selectedGoogleRoute.legs[0] && selectedGoogleRoute.legs[0].via_waypoints) {
    waypoints = selectedGoogleRoute.legs[0].via_waypoints.map(point => 
      `${point.lat()},${point.lng()}`
    );
    if (waypoints.length > 0) {
      googleMapsUrl += `&waypoints=${encodeURIComponent(waypoints.join('|'))}`;
    }
  }

  // Update UI
  const startButton = document.getElementById('start-commute-btn');
  startButton.textContent = 'Opening Google Maps...';
  startButton.style.background = '#4CAF50';
  
  // Open Google Maps in a new tab
  window.open(googleMapsUrl, '_blank');
}

async function calculateDetailedEmissions(route, mode) {
  try {
    const routeDetails = await analyzeRoute(route);
    let baseEmission = 0;
    const distanceKm = routeDetails.totalDistance;

    // Calculate base emission based on mode
    switch (mode) {
      case 'DRIVING':
        baseEmission = distanceKm * EMISSION_FACTORS.DRIVING.MEDIUM_CAR;
        break;
      case 'PUBLIC_TRANSPORT':
      case 'TRANSIT':
        // Use average public transport emission factor
        baseEmission = distanceKm * EMISSION_FACTORS.TRANSIT;
        break;
      case 'BICYCLING':
        baseEmission = distanceKm * EMISSION_FACTORS.BICYCLING;
        break;
      case 'WALKING':
        baseEmission = distanceKm * EMISSION_FACTORS.WALKING;
        break;
      default:
        baseEmission = distanceKm * EMISSION_FACTORS.DRIVING.MEDIUM_CAR;
    }

    // Apply route factors
    let totalEmission = baseEmission;

    // Apply elevation factor if available
    if (routeDetails.totalElevationGain) {
      // Add 10% more emissions per 100m elevation gain
      const elevationFactor = 1 + (routeDetails.totalElevationGain / 100) * 0.1;
      totalEmission *= elevationFactor;
    }

    // Apply traffic factor if available and if mode is affected by traffic
    if (routeDetails.averageTrafficDelay && ['DRIVING', 'TRANSIT', 'PUBLIC_TRANSPORT'].includes(mode)) {
      const trafficFactor = 1 + (routeDetails.averageTrafficDelay / 100);
      totalEmission *= trafficFactor;
    }

    return {
      totalEmission: totalEmission,
      details: {
        ...routeDetails,
        baseEmission: baseEmission,
        mode: mode,
        emissionFactors: {
          base: mode === 'DRIVING' ? EMISSION_FACTORS.DRIVING.MEDIUM_CAR :
                mode === 'TRANSIT' || mode === 'PUBLIC_TRANSPORT' ? EMISSION_FACTORS.TRANSIT :
                mode === 'BICYCLING' ? EMISSION_FACTORS.BICYCLING : 0,
          elevation: routeDetails.totalElevationGain ? 
                    1 + (routeDetails.totalElevationGain / 100) * 0.1 : 1,
          traffic: routeDetails.averageTrafficDelay ? 
                   1 + (routeDetails.averageTrafficDelay / 100) : 1
        }
      }
    };
  } catch (error) {
    console.error('Error calculating emissions:', error);
    return {
      totalEmission: 0,
      details: {
        error: 'Failed to calculate emissions'
      }
    };
  }
}

async function analyzeRoute(route) {
  const analysis = {
    segments: [],
    totalDistance: 0,
    totalDuration: 0,
    elevationProfile: [],
    trafficConditions: [],
    roadTypes: new Map(),
    averageSpeed: 0,
    trafficDelays: [],
    speedProfile: {
      averageSpeed: 0,
      maxSpeed: 0,
      minSpeed: Infinity,
      speedsByRoadType: new Map()
    }
  };

  // Analyze each route segment
  for (const leg of route.legs) {
    for (const step of leg.steps) {
      const segment = await analyzeRouteSegment(step);
      analysis.segments.push(segment);
      analysis.totalDistance += segment.distance;
      analysis.totalDuration += segment.duration;
      
      // Update speed profile
      analysis.speedProfile.maxSpeed = Math.max(analysis.speedProfile.maxSpeed, segment.actualSpeed);
      analysis.speedProfile.minSpeed = Math.min(analysis.speedProfile.minSpeed, segment.actualSpeed);
      
      // Track speeds by road type
      if (!analysis.speedProfile.speedsByRoadType.has(segment.roadType)) {
        analysis.speedProfile.speedsByRoadType.set(segment.roadType, []);
      }
      analysis.speedProfile.speedsByRoadType.get(segment.roadType).push(segment.actualSpeed);
      
      // Track traffic delays
      if (segment.trafficLevel > 1) {
        analysis.trafficDelays.push({
          location: step.start_location,
          delay: (segment.trafficLevel - 1) * 100,
          distance: segment.distance
        });
      }

      // Aggregate road type usage
      if (analysis.roadTypes.has(segment.roadType)) {
        analysis.roadTypes.set(
          segment.roadType, 
          analysis.roadTypes.get(segment.roadType) + segment.distance
        );
      } else {
        analysis.roadTypes.set(segment.roadType, segment.distance);
      }
    }
  }

  // Calculate average speeds by road type
  for (const [roadType, speeds] of analysis.speedProfile.speedsByRoadType) {
    const avgSpeed = speeds.reduce((a, b) => a + b, 0) / speeds.length;
    analysis.speedProfile.speedsByRoadType.set(roadType, avgSpeed);
  }

  // Calculate overall average speed
  analysis.speedProfile.averageSpeed = analysis.totalDistance / (analysis.totalDuration / 60);

  // Get elevation data and calculate cycling-specific speeds if needed
  try {
    const elevationData = await getRouteElevation(route);
    analysis.elevationProfile = elevationData;
    
    // Calculate elevation-based speeds for cycling
    if (route.request.travelMode === 'BICYCLING') {
      analysis.cyclingProfile = calculateCyclingProfile(elevationData);
    }
  } catch (error) {
    console.error('Error getting elevation data:', error);
  }

  return analysis;
}

async function analyzeRouteSegment(step) {
  const segment = {
    distance: step.distance.value / 1000, // Convert to km
    duration: step.duration.value / 60,   // Convert to minutes
    roadType: determineRoadType(step),
    instructions: step.instructions
  };

  // Calculate realistic speed based on road type and conditions
  const roadType = segment.roadType;
  const hasTraffic = step.duration_in_traffic && step.duration_in_traffic.value > step.duration.value;
  const speedLimit = SPEED_LIMITS[roadType] || SPEED_LIMITS.unknown;
  
  // Calculate actual speed based on distance and time
  const actualSpeed = (segment.distance / segment.duration) * 60;
  
  // Determine if there's congestion
  const isCongested = hasTraffic && actualSpeed < speedLimit.normal * 0.7;
  
  segment.speedLimit = speedLimit.normal;
  segment.actualSpeed = actualSpeed;
  segment.trafficLevel = step.duration_in_traffic ? 
    step.duration_in_traffic.value / step.duration.value : 1;
  segment.isCongested = isCongested;

  return segment;
}

function determineRoadType(step) {
  const instruction = step.instructions.toLowerCase();
  
  // More detailed road type analysis
  if (instruction.includes('motorway') || instruction.includes('highway')) {
    return 'highway';
  } else if (instruction.includes('trunk') || instruction.includes('primary')) {
    return 'major_road';
  } else if (instruction.includes('secondary') || instruction.includes('tertiary')) {
    return 'secondary_road';
  } else if (instruction.includes('residential') || instruction.includes('local')) {
    return 'residential';
  } else if (instruction.includes('service') || instruction.includes('private')) {
    return 'service_road';
  } else if (instruction.includes('toll')) {
    return 'toll_road';
  } else {
    // Try to determine road type from the HTML instructions
    const htmlInstructions = step.instructions.toLowerCase();
    if (htmlInstructions.includes('expressway') || htmlInstructions.includes('freeway')) {
      return 'highway';
    } else if (htmlInstructions.includes('main road') || htmlInstructions.includes('major')) {
      return 'major_road';
    } else if (htmlInstructions.includes('street') || htmlInstructions.includes('road')) {
      return 'secondary_road';
    }
    return 'unknown';
  }
}

async function getRouteElevation(route) {
  const path = route.legs[0].steps.map(step => ({
    lat: step.start_location.lat(),
    lng: step.start_location.lng()
  }));

  try {
    const response = await elevationService.getElevationAlongPath({
      path: path,
      samples: Math.min(512, path.length * 2) // Google's limit is 512 samples
    });

    return response.results.map((result, index) => ({
      elevation: result.elevation,
      location: result.location,
      distance: index === 0 ? 0 : 
        google.maps.geometry.spherical.computeDistanceBetween(
          response.results[index - 1].location,
          result.location
        ) / 1000 // Convert to km
    }));
  } catch (error) {
    console.error('Error getting elevation data:', error);
    return [];
  }
}

async function getTrafficConditions(route) {
  const origin = route.legs[0].start_location;
  const destination = route.legs[0].end_location;

  try {
    const response = await distanceMatrixService.getDistanceMatrix({
      origins: [origin],
      destinations: [destination],
      travelMode: google.maps.TravelMode.DRIVING,
      drivingOptions: {
        departureTime: new Date(),
        trafficModel: google.maps.TrafficModel.BEST_GUESS
      }
    });

    return {
      durationInTraffic: response.rows[0].elements[0].duration_in_traffic.value,
      duration: response.rows[0].elements[0].duration.value,
      trafficFactor: response.rows[0].elements[0].duration_in_traffic.value / 
                    response.rows[0].elements[0].duration.value
    };
  } catch (error) {
    console.error('Error getting traffic conditions:', error);
    return { trafficFactor: 1 };
  }
}

function calculateCyclingProfile(elevationProfile) {
  const profile = {
    segments: [],
    averageSpeed: 0
  };

  if (!elevationProfile || elevationProfile.length < 2) {
    return profile;
  }

  let totalDistance = 0;
  let totalTime = 0;

  for (let i = 1; i < elevationProfile.length; i++) {
    const elevation1 = elevationProfile[i - 1].elevation;
    const elevation2 = elevationProfile[i].elevation;
    const distance = elevationProfile[i].distance;
    const gradient = (elevation2 - elevation1) / (distance * 1000);

    let speed;
    if (gradient > 0.1) speed = CYCLING_SPEEDS.steep_uphill;
    else if (gradient > 0.05) speed = CYCLING_SPEEDS.moderate_uphill;
    else if (gradient > 0.02) speed = CYCLING_SPEEDS.slight_uphill;
    else if (gradient < -0.1) speed = CYCLING_SPEEDS.steep_downhill;
    else if (gradient < -0.05) speed = CYCLING_SPEEDS.moderate_downhill;
    else if (gradient < -0.02) speed = CYCLING_SPEEDS.slight_downhill;
    else speed = CYCLING_SPEEDS.flat;

    const timeHours = distance / speed;
    totalDistance += distance;
    totalTime += timeHours;

    profile.segments.push({
      distance: distance,
      gradient: gradient,
      speed: speed,
      type: Object.entries(CYCLING_SPEEDS).find(([, v]) => v === speed)[0]
    });
  }

  profile.averageSpeed = totalDistance / totalTime;
  return profile;
}

// Add after initMap function
function toggle3DMode() {
  const is3DEnabled = document.getElementById('enable3d').checked;
  if (is3DEnabled) {
    map.setTilt(45);
    map.setMapTypeId('satellite');
  } else {
    map.setTilt(0);
    map.setMapTypeId('roadmap');
  }
}

function getCurrentLocation() {
  const locationBtn = document.querySelector('.location-btn');
  const startInput = document.getElementById('start');

  // Show loading state
  locationBtn.style.opacity = '0.7';
  locationBtn.disabled = true;
  startInput.placeholder = 'Getting your location...';

  if (navigator.geolocation) {
    navigator.geolocation.getCurrentPosition(
      // Success callback
      (position) => {
        const lat = position.coords.latitude;
        const lng = position.coords.longitude;

        // Reverse geocode to get address
        geocoder.geocode(
          { location: { lat, lng } },
          (results, status) => {
            if (status === 'OK' && results[0]) {
              startInput.value = results[0].formatted_address;
              
              // Center map on current location
              map.setCenter({ lat, lng });
              map.setZoom(15);

              // Add a marker for current location
              if (window.currentLocationMarker) {
                window.currentLocationMarker.setMap(null);
              }
              window.currentLocationMarker = new google.maps.Marker({
                position: { lat, lng },
                map: map,
                title: 'Your Location',
                animation: google.maps.Animation.DROP,
                icon: {
                  path: google.maps.SymbolPath.CIRCLE,
                  scale: 10,
                  fillColor: '#4CAF50',
                  fillOpacity: 1,
                  strokeColor: 'white',
                  strokeWeight: 2,
                }
              });
            } else {
              alert('Could not find address for your location.');
              startInput.value = `${lat},${lng}`;
            }

            // Reset button state
            locationBtn.style.opacity = '1';
            locationBtn.disabled = false;
            startInput.placeholder = 'Enter start location';
          }
        );
      },
      // Error callback
      (error) => {
        let errorMessage = 'Error getting your location: ';
        switch(error.code) {
          case error.PERMISSION_DENIED:
            errorMessage += 'Location permission denied.';
            break;
          case error.POSITION_UNAVAILABLE:
            errorMessage += 'Location information unavailable.';
            break;
          case error.TIMEOUT:
            errorMessage += 'Location request timed out.';
            break;
          default:
            errorMessage += 'Unknown error occurred.';
        }
        alert(errorMessage);
        
        // Reset button state
        locationBtn.style.opacity = '1';
        locationBtn.disabled = false;
        startInput.placeholder = 'Enter start location';
      },
      // Options
      {
        enableHighAccuracy: true,
        timeout: 10000,
        maximumAge: 0
      }
    );
  } else {
    alert('Geolocation is not supported by your browser.');
    locationBtn.style.opacity = '1';
    locationBtn.disabled = false;
  }
}
