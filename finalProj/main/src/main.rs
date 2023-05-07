use std::collections::{BinaryHeap, HashMap};
use std::io::{self, Write};

// Define a struct for each T stop
#[derive(Debug)]
struct TStop {
    name: String,
    neighbors: Vec<(TStop, u32)>,
}

impl TStop {
    fn new(name: &str) -> TStop {
        TStop {
            name: name.to_string(),
            neighbors: Vec::new(),
        }
    }

    // Add a neighboring stop with its distance
    fn add_neighbor(&mut self, neighbor: (TStop, u32)) {
        self.neighbors.push(neighbor);
    }

    fn add_neighbors(&mut self, names: Vec<&str>, distances: Vec<u32>) {
        for (n, d) in names.iter().zip(distances) {
            self.add_neighbor((TStop::new(n), d));
        }
    }
}

fn shortest_distance(start: &str, end: &str, stops: &HashMap<String, TStop>) -> Option<(u32, Vec<String>)> {
    let mut distances: HashMap<String, u32> = HashMap::new();
    let mut prev_stops: HashMap<String, String> = HashMap::new();
    let mut heap = BinaryHeap::new();

    // Set all distances to infinity except the starting stop
    for (name, _) in stops {
        if name == start {
            distances.insert(name.to_string(), 0);
            heap.push((0, name.to_string()));
        } else {
            distances.insert(name.to_string(), u32::max_value());
        }
    }

    while let Some((distance, current_stop)) = heap.pop() {
        // If we've reached the end stop, return its distance and path
        if current_stop == end {
            let mut path = Vec::new();
            let mut stop = &current_stop;
            while stop != start {
                path.push(stop.to_owned());
                stop = prev_stops.get(stop).unwrap();
            }
            path.push(start.to_owned());
            path.reverse();
            return Some((distance, path));
        }

        // Otherwise, update distances to neighboring stops
        for (neighbor, neighbor_distance) in &stops[&current_stop].neighbors {
            let total_distance = distance + neighbor_distance;
            if total_distance < distances[&neighbor.name] {
                distances.insert(neighbor.name.to_string(), total_distance);
                prev_stops.insert(neighbor.name.to_string(), current_stop.to_string());
                heap.push((total_distance, neighbor.name.to_string()));
            }
        }
    }

    // If we've exhausted all stops without reaching the end, return None
    None
}

fn main() {
    // Create T stops and their neighbors with distances
    let mut stops: HashMap<String, TStop> = HashMap::new();
    
    let mut alewife: TStop = TStop::new("Alewife");
    alewife.add_neighbors(vec!["Davis"], vec![1580]);
    stops.insert("Alewife".to_string(), alewife);

    let mut davis: TStop = TStop::new("Davis");
    davis.add_neighbors(vec!["Alewife","Porter"], vec![1580,964]);
    stops.insert("Davis".to_string(), davis);

    let mut porter: TStop = TStop::new("Porter");
    porter.add_neighbors(vec!["Davis","Harvard"], vec![964,1650]);
    stops.insert("Porter".to_string(), porter);

    let mut harvard_a: TStop = TStop::new("Harvard");
    harvard_a.add_neighbors(vec!["Porter","Central"], vec![1650,1600]);
    stops.insert("Harvard".to_string(), harvard_a);

    let mut central: TStop = TStop::new("Central");
    central.add_neighbors(vec!["Harvard","Kendall/MIT"], vec![1600,1520]);
    stops.insert("Central".to_string(), central);

    let mut kendall_mit: TStop = TStop::new("Kendall/MIT");
    kendall_mit.add_neighbors(vec!["Central","Charles/MGH"], vec![1520,1230]);
    stops.insert("Kendall/MIT".to_string(), kendall_mit);

    let mut charles_mgh: TStop = TStop::new("Charles/MGH");
    charles_mgh.add_neighbors(vec!["Kendall/MIT","Park St"], vec![1230,867]);
    stops.insert("Charles/MGH".to_string(), charles_mgh);

    let mut park_st: TStop = TStop::new("Park St");
    park_st.add_neighbors(vec!["Charles/MGH","Downtown Crossing", "Boylston","Gov't Center"], vec![867,202,446,440]);
    stops.insert("Park St".to_string(), park_st);

    let mut downtown_crossing: TStop = TStop::new("Downtown Crossing");
    downtown_crossing.add_neighbors(vec!["Park St","South Station","Chinatown","State"], vec![202,562,405,437]);
    stops.insert("Downtown Crossing".to_string(), downtown_crossing);

    let mut south_station: TStop = TStop::new("South Station");
    south_station.add_neighbors(vec!["Downtown Crossing","Broadway"], vec![562,1180]);
    stops.insert("South Station".to_string(), south_station);

    let mut broadway: TStop = TStop::new("Broadway");
    broadway.add_neighbors(vec!["South Station","Andrew"], vec![1180,1410]);
    stops.insert("Broadway".to_string(), broadway);

    let mut andrew: TStop = TStop::new("Andrew");
    andrew.add_neighbors(vec!["Broadway","JFK/UMass"], vec![1410,1120]);
    stops.insert("Andrew".to_string(), andrew);

    let mut jfk_umass: TStop = TStop::new("JFK/UMass");
    jfk_umass.add_neighbors(vec!["Andrew","Savin Hill","North Quincy"], vec![1120,1050,5370]);
    stops.insert("JFK/UMass".to_string(), jfk_umass);


// First red line split
    let mut savin_hill: TStop = TStop::new("Savin Hill");
    savin_hill.add_neighbors(vec!["JFK/UMass","Fields Corner"], vec![1050,1440]);
    stops.insert("Savin Hill".to_string(), savin_hill);
    
    let mut fields_corner: TStop = TStop::new("Fields Corner");
    fields_corner.add_neighbors(vec!["Savin Hill","Shawmut"], vec![1440,850]);
    stops.insert("Fields Corner".to_string(), fields_corner);

    let mut shawmut: TStop = TStop::new("Shawmut");
    shawmut.add_neighbors(vec!["Fields Corner","Ashmont"], vec![850,983]);
    stops.insert("Shawmut".to_string(), shawmut);

    let mut ashmont: TStop = TStop::new("Ashmont");
    ashmont.add_neighbors(vec!["Shawmut","Cedar Grove"], vec![983,625]);
    stops.insert("Ashmont".to_string(), ashmont);

    let mut cedar_grove: TStop = TStop::new("Cedar Grove");
    cedar_grove.add_neighbors(vec!["Ashmont","Butler"], vec![625,813]);
    stops.insert("Cedar Grove".to_string(), cedar_grove);

    let mut butler: TStop = TStop::new("Butler");
    butler.add_neighbors(vec!["Cedar Grove","Milton"], vec![813,447]);
    stops.insert("Butler".to_string(), butler);

    let mut milton: TStop = TStop::new("Milton");
    milton.add_neighbors(vec!["Butler","Central Ave"], vec![447,511]);
    stops.insert("Milton".to_string(), milton);

    let mut central_ave: TStop = TStop::new("Central Ave");
    central_ave.add_neighbors(vec!["Milton","Valley Rd"], vec![511,674]);
    stops.insert("Central Ave".to_string(), central_ave);

    let mut valley_rd: TStop = TStop::new("Valley Rd");
    valley_rd.add_neighbors(vec!["Central Ave","Capen St"], vec![511,467]);
    stops.insert("Valley Rd".to_string(), valley_rd);

    let mut capen_st: TStop = TStop::new("Capen St");
    capen_st.add_neighbors(vec!["Valley Rd","Mattapan"], vec![467,445]);
    stops.insert("Capen St".to_string(), capen_st);

    let mut mattapan: TStop = TStop::new("Mattapan");
    mattapan.add_neighbors(vec!["Capen St"], vec![445]);
    stops.insert("Mattapan".to_string(), mattapan);

// Second Red split 
    let mut north_quincy: TStop = TStop::new("North Quincy");
    north_quincy.add_neighbors(vec!["JFK/UMass","Wollaston"], vec![5370,1240]);
    stops.insert("North Quincy".to_string(), north_quincy);

    let mut wollaston: TStop = TStop::new("Wollaston");
    wollaston.add_neighbors(vec!["North Quincy","Quincy Center"], vec![1240,2020]);
    stops.insert("Wollaston".to_string(), wollaston);

    let mut quincy_center: TStop = TStop::new("Quincy Center");
    quincy_center.add_neighbors(vec!["Wollaston","Quincy Adams"], vec![2020,2110]);
    stops.insert("Quincy Center".to_string(), quincy_center);

    let mut quincy_adams: TStop = TStop::new("Quincy Adams");
    quincy_adams.add_neighbors(vec!["Quincy Center","Braintree"], vec![2110,2880]);
    stops.insert("Quincy Adams".to_string(), quincy_adams);

    let mut braintree: TStop = TStop::new("Braintree");
    braintree.add_neighbors(vec!["Quincy Adams"], vec![2880]);
    stops.insert("Braintree".to_string(), braintree);

    // Green Line left

    let mut boylston: TStop = TStop::new("Boylston");
    boylston.add_neighbors(vec!["Park St","Arlington"], vec![460,519]);
    stops.insert("Boylston".to_string(), boylston);

    let mut arlington: TStop = TStop::new("Arlington");
    arlington.add_neighbors(vec!["Boylston","Copley"], vec![519,565]);
    stops.insert("Arlington".to_string(), arlington);

    let mut copley: TStop = TStop::new("Copley");
    copley.add_neighbors(vec!["Arlington","Hynes Convention Center","Prudential"], vec![565, 1070,632]);
    stops.insert("Copley".to_string(), copley);

    let mut hynes_convention: TStop = TStop::new("Hynes Convention Center");
    hynes_convention.add_neighbors(vec!["Copley","Kenmore"], vec![1070, 601]);
    stops.insert("Hynes Convention Center".to_string(), hynes_convention);

    let mut kenmore: TStop = TStop::new("Kenmroe");
    kenmore.add_neighbors(vec!["Hynes Convention Center","Blandford St","St. Mary's St", "Fenway"], vec![601,418,1020,840]);
    stops.insert("Kenmore".to_string(), kenmore);

// B Split
    let mut blandford_st: TStop = TStop::new("Blandford St");
    blandford_st.add_neighbors(vec!["Kenmore","BU East"], vec![418, 331]);
    stops.insert("Blandford St".to_string(), blandford_st);

    let mut bu_east: TStop = TStop::new("BU East");
    bu_east.add_neighbors(vec!["Blandford St","BU Central"], vec![418, 241]);
    stops.insert("BU East".to_string(), bu_east);

    let mut bu_central: TStop = TStop::new("BU Central");
    bu_central.add_neighbors(vec!["BU East","Amory St"], vec![241, 634]);
    stops.insert("BU Central".to_string(), bu_central);

    let mut amory_st: TStop = TStop::new("Amory St");
    amory_st.add_neighbors(vec!["BU Central","Babcock St"], vec![634, 425]);
    stops.insert("Amory St".to_string(), amory_st);

    let mut babcock: TStop = TStop::new("Babcock St");
    babcock.add_neighbors(vec!["Amory St","Packard's Corner"], vec![425,430]);
    stops.insert("Babcock St".to_string(), babcock);

    let mut packard: TStop = TStop::new("Packard's Corner");
    packard.add_neighbors(vec!["Amory St","Harvard Ave"], vec![430,459]);
    stops.insert("Packard's Corner".to_string(), packard);

    let mut harvard: TStop = TStop::new("Harvard Ave");
    harvard.add_neighbors(vec!["Packard's Corner","Griggs St"], vec![459,413]);
    stops.insert("Harvard Ave".to_string(), harvard);

    let mut griggs: TStop = TStop::new("Griggs St");
    griggs.add_neighbors(vec!["Harvard Ave","Allston St"], vec![413,259]);
    stops.insert("Griggs St".to_string(), griggs);

    let mut allston: TStop = TStop::new("Allston St");
    allston.add_neighbors(vec!["Griggs St", "Warren St"], vec![259,208]);
    stops.insert("Allston St".to_string(), allston);

    let mut allston: TStop = TStop::new("Allston St");
    allston.add_neighbors(vec!["Griggs St", "Warren St"], vec![259,208]);
    stops.insert("Allston St".to_string(), allston);

    let mut warren: TStop = TStop::new("Warren St");
    warren.add_neighbors(vec!["Allston St", "Washington St"], vec![208,497]);
    stops.insert("Warren St".to_string(), warren);

    let mut washington: TStop = TStop::new("Washington St");
    washington.add_neighbors(vec!["Warren St", "Sutherland Rd"], vec![497,417]);
    stops.insert("Washington St".to_string(), washington);

    let mut sutherland: TStop = TStop::new("Sutherland Rd");
    sutherland.add_neighbors(vec!["Washington St", "Chiswick Rd"], vec![417,377]);
    stops.insert("Sutherland Rd".to_string(), sutherland);

    let mut chiswick: TStop = TStop::new("Chiswick Rd");
    chiswick.add_neighbors(vec!["Sutherland Rd","Chesnut Hill Ave"], vec![377,347]);
    stops.insert("Chiswick Rd".to_string(), chiswick);

    let mut chesnut: TStop = TStop::new("Chesnut Hill Ave");
    chesnut.add_neighbors(vec!["Chiswick Rd","South St"], vec![347,406]);
    stops.insert("Chesnut Hill Ave".to_string(), chesnut);

    let mut south_st: TStop = TStop::new("South St");
    south_st.add_neighbors(vec!["Chesnut Hill Ave","Boston College"], vec![406,786]);
    stops.insert("South St".to_string(), south_st);

    let mut boston_college: TStop = TStop::new("Boston College");
    boston_college.add_neighbors(vec!["South St"], vec![786]);
    stops.insert("Boston College".to_string(), boston_college);

// C split

    let mut stmary: TStop = TStop::new("St. Mary's St");
    stmary.add_neighbors(vec!["Kenmore","Hawes St"], vec![1020, 331]);
    stops.insert("St. Mary's St".to_string(), stmary);

    let mut hawes: TStop = TStop::new("Hawes St");
    hawes.add_neighbors(vec!["St. Mary's St","Kent St"], vec![331, 250]);
    stops.insert("Hawes St".to_string(), hawes);

    let mut kent: TStop = TStop::new("Kent St");
    kent.add_neighbors(vec!["Hawes St","St. Paul St"], vec![250, 233]);
    stops.insert("Kent St".to_string(), kent);

    let mut stpaul: TStop = TStop::new("St. Paul St");
    stpaul.add_neighbors(vec!["Kent St","Coolidge Corner"], vec![233, 374]);
    stops.insert("St. Paul St".to_string(),stpaul);

    let mut coolidge: TStop = TStop::new("Coolidge Corner");
    coolidge.add_neighbors(vec!["St. Paul St","Summit Ave"], vec![374, 387]);
    stops.insert("Coolidge Corner".to_string(),coolidge);

    let mut summit: TStop = TStop::new("Summit Ave");
    summit.add_neighbors(vec!["Coolidge Corner","Brandon Hall"], vec![374, 266]);
    stops.insert("Summit Ave".to_string(),summit);

    let mut brandon: TStop = TStop::new("Brandon Hall");
    brandon.add_neighbors(vec!["Summit Ave","Fairbanks St"], vec![266, 214]);
    stops.insert("Brandon Hall".to_string(),brandon);

    let mut fairbank: TStop = TStop::new("Fairbanks St");
    fairbank.add_neighbors(vec!["Summit Ave","Washington Sq"], vec![214, 333]);
    stops.insert("Fairbanks St".to_string(),fairbank);

    let mut washington_sq: TStop = TStop::new("Washington Sq");
    washington_sq.add_neighbors(vec!["Fairbanks St","Tappan St"], vec![333, 263]);
    stops.insert("Washington Sq".to_string(),washington_sq);

    let mut tappan: TStop = TStop::new("Tappan St");
    tappan.add_neighbors(vec!["Washington Sq", "Dean Rd"], vec![263,319]);
    stops.insert("Tappan St".to_string(),tappan);

    let mut dean: TStop = TStop::new("Dean Rd");
    dean.add_neighbors(vec!["Tappan St", "Englewood Ave"], vec![319,262]);
    stops.insert("Dean Rd".to_string(),dean);

    let mut englewood: TStop = TStop::new("Englewood Ave");
    englewood.add_neighbors(vec!["Dean Rd", "Cleveland Circle"], vec![262, 318]);
    stops.insert("Englewood Ave".to_string(),englewood);

    let mut cleveland: TStop = TStop::new("Cleveland Circle");
    cleveland.add_neighbors(vec!["Englewood Ave"], vec![318]);
    stops.insert("Cleveland Circle".to_string(),cleveland);

    // D Branch

    let mut fenway: TStop = TStop::new("Fenway");
    fenway.add_neighbors(vec!["Kenmore","Longwood"], vec![840,641]);
    stops.insert("Fenway".to_string(),fenway);

    let mut longwood: TStop = TStop::new("Longwood");
    longwood.add_neighbors(vec!["Fenway","Brookline Village"], vec![641,1140]);
    stops.insert("Longwood".to_string(),longwood);

    let mut brookline_vil: TStop = TStop::new("Brookline Village");
    brookline_vil.add_neighbors(vec!["Longwood","Brookline Hills"], vec![1140, 843]);
    stops.insert("Brookline Village".to_string(),brookline_vil);

    let mut brookline_hill: TStop = TStop::new("Brookline Hills");
    brookline_hill.add_neighbors(vec!["Brookline Village","Beaconsfield"], vec![843,1240]);
    stops.insert("Brookline Hills".to_string(),brookline_hill);

    let mut beaconsfield: TStop = TStop::new("Beaconsfield");
    beaconsfield.add_neighbors(vec!["Brookline Hills","Reservoir"], vec![1240,689]);
    stops.insert("Beaconsfield".to_string(),beaconsfield);

    let mut reservoir: TStop = TStop::new("Reservoir");
    reservoir.add_neighbors(vec!["Beaconsfield","Chesnut Hill"], vec![689, 1620]);
    stops.insert("Reservoir".to_string(),reservoir);

    let mut chesnut_hill: TStop = TStop::new("Chesnut Hill");
    chesnut_hill.add_neighbors(vec!["Reservoir","Newton Centre"], vec![1620,2300]);
    stops.insert("Chesnut Hill".to_string(),chesnut_hill);

    let mut newton_centre: TStop = TStop::new("Newton Centre");
    newton_centre.add_neighbors(vec!["Chesnut Hill","Newton Highlands"], vec![2300,1330]);
    stops.insert("Newton Centre".to_string(),newton_centre);

    let mut newton_highlands: TStop = TStop::new("Newton Highlands");
    newton_highlands.add_neighbors(vec!["Newton Centre","Eliot"], vec![1330,987]);
    stops.insert("Newton Highlands".to_string(),newton_highlands);

    let mut eliot: TStop = TStop::new("Eliot");
    eliot.add_neighbors(vec!["Newton Highlands","Waban"], vec![1330,1360]);
    stops.insert("Eliot".to_string(),eliot);

    let mut waban: TStop = TStop::new("Waban");
    waban.add_neighbors(vec!["Eliot","Woodland"], vec![1360,1280]);
    stops.insert("Waban".to_string(),waban);

    let mut woodland: TStop = TStop::new("Woodland");
    woodland.add_neighbors(vec!["Waban","Riverside"], vec![1280,933]);
    stops.insert("Woodland".to_string(),woodland);

    let mut riverside: TStop = TStop::new("Riverside");
    riverside.add_neighbors(vec!["Woodland"], vec![933]);
    stops.insert("Riverside".to_string(),riverside);

    // E split

    let mut prudential: TStop = TStop::new("Prudential");
    prudential.add_neighbors(vec!["Copley","Symphony"], vec![632,460]);
    stops.insert("Prudential".to_string(),prudential);

    let mut symphony: TStop = TStop::new("Symphony");
    symphony.add_neighbors(vec!["Prudential","Northeastern"], vec![460,1612]);
    stops.insert("Symphony".to_string(),symphony);

    let mut northeastern: TStop = TStop::new("Northeastern");
    northeastern.add_neighbors(vec!["Symphony","Museum of Fine Arts"], vec![1612, 494]);
    stops.insert("Northeastern".to_string(),northeastern);

    let mut museum: TStop = TStop::new("Museum of Fine Arts");
    museum.add_neighbors(vec!["Northeastern","Longwood Medical Area"], vec![494,427]);
    stops.insert("Museum of Fine Arts".to_string(),museum);

    let mut longwood_med: TStop = TStop::new("Longwood Medical Area");
    longwood_med.add_neighbors(vec!["Museum of Fine Arts","Brigham Circle"], vec![427,346]);
    stops.insert("Longwood Medical Area".to_string(),longwood_med);

    let mut brigham: TStop = TStop::new("Brigham Circle");
    brigham.add_neighbors(vec!["Longwood Medical Area", "Fenwood Rd"], vec![346,233]);
    stops.insert("Brigham Circle".to_string(),brigham);

    let mut fenwood: TStop = TStop::new("Fenwood Rd");
    fenwood.add_neighbors(vec!["Brigham Circle","Mission Park"], vec![233,264]);
    stops.insert("Fenwood Rd".to_string(),fenwood);

    let mut mission: TStop = TStop::new("Mission Park");
    mission.add_neighbors(vec!["Fenwood Rd","Riverway"], vec![264, 305]);
    stops.insert("Mission Park".to_string(),mission);

    let mut riverway: TStop = TStop::new("Riverway");
    riverway.add_neighbors(vec!["Mission Park","Back of the Hill"], vec![305,245]);
    stops.insert("Riverway".to_string(),riverway);

    let mut back: TStop = TStop::new("Back of the Hill");
    back.add_neighbors(vec!["Riverway","Heath St"], vec![305,120]);
    stops.insert("Back of the Hill".to_string(),back);

    let mut heath: TStop = TStop::new("Heath St");
    heath.add_neighbors(vec!["Back of the Hill"], vec![120]);
    stops.insert("Heath St".to_string(),heath);

// Green line other side of park st
    let mut govt: TStop = TStop::new("Gov't Center");
    govt.add_neighbors(vec!["Park St","Haymarket","Bowdoin","State"], vec![440,369,290,180]);
    stops.insert("Gov't Center".to_string(),govt);

    let mut hay: TStop = TStop::new("Haymarket");
    hay.add_neighbors(vec!["Gov't Center", "North Station","State"], vec![440,369,454]);
    stops.insert("Haymarket".to_string(),hay);

    let mut north: TStop = TStop::new("North Station");
    north.add_neighbors(vec!["Haymarket", "Science Park/West End", "Community College"], vec![369,713,1030]);
    stops.insert("North Station".to_string(),north);

    let mut science: TStop = TStop::new("Science Park/West End");
    science.add_neighbors(vec!["North Station", "Lechmere"], vec![713,879]);
    stops.insert("Science Park/West End".to_string(),science);

    let mut lechmere: TStop = TStop::new("Lechmere");
    lechmere.add_neighbors(vec!["Science Park/West End", "Union Sq", "East Somerville"], vec![713,1640,1180]);
    stops.insert("Lechmere".to_string(),lechmere);

    let mut union_sq: TStop = TStop::new("Union Sq");
    union_sq.add_neighbors(vec!["Lechmere"], vec![1640]);
    stops.insert("Union Sq".to_string(),union_sq);

    let mut east_s: TStop = TStop::new("East Somerville");
    east_s.add_neighbors(vec!["Lechmere", "Gilman Sq"], vec![1180, 1260]);
    stops.insert("East Somerville".to_string(),east_s);

    let mut gilman: TStop = TStop::new("Gilman Sq");
    gilman.add_neighbors(vec!["East Somerville", "Magoun Sq"], vec![1260,1000]);
    stops.insert("Gilman Sq".to_string(),gilman);

    let mut magoun: TStop = TStop::new("Magoun Sq");
    magoun.add_neighbors(vec!["Gilman Sq", "Ball Sq"], vec![1000,780]);
    stops.insert("Magoun Sq".to_string(),magoun);

    let mut ball: TStop = TStop::new("Ball Sq");
    ball.add_neighbors(vec!["Magoun Sq", "Medford/Tufts"], vec![780,1000]);
    stops.insert("Ball Sq".to_string(),ball);

    let mut tufts: TStop = TStop::new("Medford/Tufts");
    tufts.add_neighbors(vec!["Ball Sq"], vec![1000]);
    stops.insert("Medford/Tufts".to_string(),tufts);

// Blue line

    let mut bowdoin: TStop = TStop::new("Bowdoin");
    bowdoin.add_neighbors(vec!["Gov't Center"], vec![290]);
    stops.insert("Bowdoin".to_string(),bowdoin);

    let mut state: TStop = TStop::new("State");
    state.add_neighbors(vec!["Gov't Center","Aquarium","Downtown Crossing", "Haymarket"], vec![180,507,437,454]);
    stops.insert("State".to_string(),state);

    let mut aquarium: TStop = TStop::new("Aquarium");
    aquarium.add_neighbors(vec!["State", "Maverick"], vec![507,1450]);
    stops.insert("Aquarium".to_string(),aquarium);

    let mut maverick: TStop = TStop::new("Maverick");
    maverick.add_neighbors(vec!["Aquarium", "Airport"], vec![1450,957]);
    stops.insert("Maverick".to_string(),maverick);

    let mut airport: TStop = TStop::new("Airport");
    airport.add_neighbors(vec!["Maverick", "Wood Island"], vec![957,863]);
    stops.insert("Airport".to_string(),airport);

    let mut wood_isl: TStop = TStop::new("Wood Island");
    wood_isl.add_neighbors(vec!["Airport", "Orient Heights"], vec![863,1690]);
    stops.insert("Wood Island".to_string(),wood_isl);

    let mut orient: TStop = TStop::new("Orient Heights");
    orient.add_neighbors(vec!["Wood Island","Suffolk Downs"],vec![1690,743]);
    stops.insert("Orient Heights".to_string(),orient);

    let mut suffolk: TStop = TStop::new("Suffolk Downs");
    suffolk.add_neighbors(vec!["Orient Heights","Beachmont"],vec![743,871]);
    stops.insert("Suffolk Downs".to_string(),suffolk);

    let mut beach: TStop = TStop::new("Beachmont");
    beach.add_neighbors(vec!["Suffolk Downs","Revere Beach"],vec![871,1160]);
    stops.insert("Beachmont".to_string(),beach);

    let mut revere: TStop = TStop::new("Revere Beach");
    revere.add_neighbors(vec!["Beachmont","Wonderland"],vec![1160,665]);
    stops.insert("Revere Beach".to_string(),revere);

    let mut wonderland: TStop = TStop::new("Wonderland");
    wonderland.add_neighbors(vec!["Revere Beach"],vec![665]);
    stops.insert("Wonderland".to_string(),wonderland);

    // Orange Line

    let mut forest: TStop = TStop::new("Forest Hills");
    forest.add_neighbors(vec!["Green St"],vec![1210]);
    stops.insert("Forest Hills".to_string(),forest);

    let mut green: TStop = TStop::new("Green St");
    green.add_neighbors(vec!["Forest Hills","Stony Brook"],vec![1210,776]);
    stops.insert("Green St".to_string(),green);

    let mut stony: TStop = TStop::new("Stony Brook");
    stony.add_neighbors(vec!["Green St","Jackson Sq"],vec![776,769]);
    stops.insert("Stony Brook".to_string(),stony);

    let mut jackson: TStop = TStop::new("Jackson Sq");
    jackson.add_neighbors(vec!["Stony Brook","Roxbury Crossing"],vec![769,973]);
    stops.insert("Jackson Sq".to_string(),jackson);

    let mut roxbury: TStop = TStop::new("Roxbury Crossing");
    roxbury.add_neighbors(vec!["Jackson Sq","Ruggles"],vec![973,810]);
    stops.insert("Roxbury Crossing".to_string(),roxbury);

    let mut ruggles: TStop = TStop::new("Ruggles");
    ruggles.add_neighbors(vec!["Roxbury Crossing","Mass. Ave"],vec![810,690]);
    stops.insert("Ruggles".to_string(),ruggles);

    let mut mass: TStop = TStop::new("Mass. Ave");
    mass.add_neighbors(vec!["Ruggles","Back Bay"],vec![690,916]);
    stops.insert("Mass. Ave".to_string(),mass);

    let mut back: TStop = TStop::new("Back Bay");
    back.add_neighbors(vec!["Mass. Ave","Tufts Medical Center"],vec![916,1000]);
    stops.insert("Back Bay".to_string(),back);

    let mut tufts_med: TStop = TStop::new("Tufts Medical Center");
    tufts_med.add_neighbors(vec!["Back Bay","Chinatown"],vec![1000,316]);
    stops.insert("Tufts Medical Center".to_string(),tufts_med);

    let mut chinatown: TStop = TStop::new("Chinatown");
    chinatown.add_neighbors(vec!["Tufts Medical Center","Downtown Crossing"],vec![316,405]);
    stops.insert("Chinatown".to_string(),chinatown);

    // some quick changes above on hay market, state, north

    let mut community: TStop = TStop::new("Community College");
    community.add_neighbors(vec!["North Station","Sullivan Sq"],vec![1030,1300]);
    stops.insert("Community College".to_string(),community);

    let mut sullivan: TStop = TStop::new("Sulivan Sq");
    sullivan.add_neighbors(vec!["Community College","Assembly"],vec![1300,982]);
    stops.insert("Sullivan Sq".to_string(),sullivan);

    let mut assembly: TStop = TStop::new("Assembly");
    assembly.add_neighbors(vec!["Sullivan Sq","Wellington"],vec![982,1060]);
    stops.insert("Assembly".to_string(),assembly);

    let mut wellington: TStop = TStop::new("Wellington");
    wellington.add_neighbors(vec!["Assembly","Malden Center"],vec![1060,2730]);
    stops.insert("Wellington".to_string(),wellington);

    let mut malden: TStop = TStop::new("Malden Center");
    malden.add_neighbors(vec!["Wellington","Oak Grove"],vec![2730,1160]);
    stops.insert("Malden Center".to_string(),malden);

    let mut oak: TStop = TStop::new("Oak Grove");
    oak.add_neighbors(vec!["Malden Center"],vec![1160]);
    stops.insert("Oak Grove".to_string(),oak);

    // Ask the user for the start and end stops
    print!("Enter the start stop: ");
    io::stdout().flush().unwrap();
    let mut start = String::new();
    io::stdin().read_line(&mut start).unwrap();
    let start = start.trim();

    print!("Enter the end stop: ");
    io::stdout().flush().unwrap();
    let mut end = String::new();
    io::stdin().read_line(&mut end).unwrap();
    let end = end.trim();

    // Check if start and end stops exist
    if !stops.contains_key(start) {
        println!("{} does not exist", start);
        return;
    }
    if !stops.contains_key(end) {
        println!("{} does not exist", end);
        return;
    }

    // Find the shortest distance between start and end stops
    match shortest_distance(start, end, &stops) {
        // Some(distance) => println!("The shortest distance between {} and {} is {}", start, end, distance),
        Some(distance) => println!("The shortest distance between {} and {} has distance in meters and path from start to finish {:?}", start, end, distance),
        None => println!("There is no path between {} and {}", start, end),
    }
} 

#[test]
fn test_shortest_distance() {
    let mut stops: HashMap<String, TStop> = HashMap::new();

    // Construct the stops and their connections
    let mut alewife: TStop = TStop::new("Alewife");
    alewife.add_neighbors(vec!["Davis"], vec![1580]);
    stops.insert("Alewife".to_string(), alewife);

    let mut davis: TStop = TStop::new("Davis");
    davis.add_neighbors(vec!["Alewife","Porter"], vec![1580,964]);
    stops.insert("Davis".to_string(), davis);

    let mut porter: TStop = TStop::new("Porter");
    porter.add_neighbors(vec!["Davis","Harvard"], vec![964,1650]);
    stops.insert("Porter".to_string(), porter);

    let mut harvard_a: TStop = TStop::new("Harvard");
    harvard_a.add_neighbors(vec!["Porter","Central"], vec![1650,1600]);
    stops.insert("Harvard".to_string(), harvard_a);

    let mut central: TStop = TStop::new("Central");
    central.add_neighbors(vec!["Harvard","Kendall/MIT"], vec![1600,1520]);
    stops.insert("Central".to_string(), central);

    let mut kendall_mit: TStop = TStop::new("Kendall/MIT");
    kendall_mit.add_neighbors(vec!["Central","Charles/MGH"], vec![1520,1230]);
    stops.insert("Kendall/MIT".to_string(), kendall_mit);

    let mut charles_mgh: TStop = TStop::new("Charles/MGH");
    charles_mgh.add_neighbors(vec!["Kendall/MIT","Park St"], vec![1230,867]);
    stops.insert("Charles/MGH".to_string(), charles_mgh);

    let mut park_st: TStop = TStop::new("Park St");
    park_st.add_neighbors(vec!["Charles/MGH","Downtown Crossing"], vec![867,202]);
    stops.insert("Park St".to_string(), park_st);

    let mut downtown_crossing: TStop = TStop::new("Downtown Crossing");
    downtown_crossing.add_neighbors(vec!["Park St","South Station"], vec![202,562]);
    stops.insert("Downtown Crossing".to_string(), downtown_crossing);

    let mut south_station: TStop = TStop::new("South Station");
    south_station.add_neighbors(vec!["Downtown Crossing"], vec![562]);
    stops.insert("South Station".to_string(), south_station);

    // Test the shortest distance between Alewife and South Station
    let result = shortest_distance("Alewife", "South Station", &stops);

    // Check that the result is correct
    assert!(result.is_some());
    let (distance, path) = result.unwrap();
    assert_eq!(distance, 10175);
    assert_eq!(path, vec!["Alewife", "Davis", "Porter", "Harvard", "Central", "Kendall/MIT", "Charles/MGH", "Park St", "Downtown Crossing", "South Station"]);
}


#[test]

fn shortest2() {
    let mut stops: HashMap<String, TStop> = HashMap::new();

    let mut park_st: TStop = TStop::new("Park St");
    park_st.add_neighbors(vec!["Gov't Center"], vec![440]);
    stops.insert("Park St".to_string(), park_st);


    let mut bowdoin: TStop = TStop::new("Bowdoin");
    bowdoin.add_neighbors(vec!["Gov't Center"], vec![290]);
    stops.insert("Bowdoin".to_string(),bowdoin);

    let mut govt: TStop = TStop::new("Gov't Center");
    govt.add_neighbors(vec!["Park St","Bowdoin"], vec![440,290]);
    stops.insert("Gov't Center".to_string(),govt);

    let result = shortest_distance("Park St", "Bowdoin", &stops);
    // Check that the result is correct
    assert!(result.is_some());
    let (distance, path) = result.unwrap();
    assert_eq!(distance, 730);
    assert_eq!(path, vec!["Park St", "Gov't Center", "Bowdoin"]);
    }

#[test]
fn shortest3() {
    let mut stops: HashMap<String, TStop> = HashMap::new();

    let mut community: TStop = TStop::new("Community College");
    community.add_neighbors(vec!["North Station","Sullivan Sq"],vec![1030,1300]);
    stops.insert("Community College".to_string(),community);

    let mut sullivan: TStop = TStop::new("Sulivan Sq");
    sullivan.add_neighbors(vec!["Community College","Assembly"],vec![1300,982]);
    stops.insert("Sullivan Sq".to_string(),sullivan);

    let mut assembly: TStop = TStop::new("Assembly");
    assembly.add_neighbors(vec!["Sullivan Sq","Wellington"],vec![982,1060]);
    stops.insert("Assembly".to_string(),assembly);

    let mut wellington: TStop = TStop::new("Wellington");
    wellington.add_neighbors(vec!["Assembly","Malden Center"],vec![1060,2730]);
    stops.insert("Wellington".to_string(),wellington);

    let mut malden: TStop = TStop::new("Malden Center");
    malden.add_neighbors(vec!["Wellington","Oak Grove"],vec![2730,1160]);
    stops.insert("Malden Center".to_string(),malden);

    let mut oak: TStop = TStop::new("Oak Grove");
    oak.add_neighbors(vec!["Malden Center"],vec![1160]);
    stops.insert("Oak Grove".to_string(),oak);
    
    let mut north: TStop = TStop::new("North Station");
    north.add_neighbors(vec!["Science Park/West End", "Community College"], vec![713,1030]);
    stops.insert("North Station".to_string(),north);

    let mut science: TStop = TStop::new("Science Park/West End");
    science.add_neighbors(vec!["North Station", "Lechmere"], vec![713,879]);
    stops.insert("Science Park/West End".to_string(),science);

    let mut lechmere: TStop = TStop::new("Lechmere");
    lechmere.add_neighbors(vec!["Science Park/West End", "Union Sq", "East Somerville"], vec![713,1640,1180]);
    stops.insert("Lechmere".to_string(),lechmere);

    let mut union_sq: TStop = TStop::new("Union Sq");
    union_sq.add_neighbors(vec!["Lechmere"], vec![1640]);
    stops.insert("Union Sq".to_string(),union_sq);

    let mut east_s: TStop = TStop::new("East Somerville");
    east_s.add_neighbors(vec!["Lechmere", "Gilman Sq"], vec![1180, 1260]);
    stops.insert("East Somerville".to_string(),east_s);

    let mut gilman: TStop = TStop::new("Gilman Sq");
    gilman.add_neighbors(vec!["East Somerville", "Magoun Sq"], vec![1260,1000]);
    stops.insert("Gilman Sq".to_string(),gilman);

    let mut magoun: TStop = TStop::new("Magoun Sq");
    magoun.add_neighbors(vec!["Gilman Sq", "Ball Sq"], vec![1000,780]);
    stops.insert("Magoun Sq".to_string(),magoun);

    let mut ball: TStop = TStop::new("Ball Sq");
    ball.add_neighbors(vec!["Magoun Sq", "Medford/Tufts"], vec![780,1000]);
    stops.insert("Ball Sq".to_string(),ball);

    let mut tufts: TStop = TStop::new("Medford/Tufts");
    tufts.add_neighbors(vec!["Ball Sq"], vec![1000]);
    stops.insert("Medford/Tufts".to_string(),tufts);


    let result = shortest_distance("Oak Grove", "Medford/Tufts", &stops);
    // Check that the result is correct
    assert!(result.is_some());
    let (distance, path) = result.unwrap();
    assert_eq!(distance, 15074);
    assert_eq!(path, vec!["Oak Grove", "Malden Center", "Wellington", "Assembly", "Sullivan Sq", "Community College", 
    "North Station", "Science Park/West End", "Lechmere", 
    "East Somerville", "Gilman Sq", "Magoun Sq", "Ball Sq", "Medford/Tufts"]);
}

