var instances = {{instances}}
var slides    = {{slides}}

/* LOAD SLIDES */
var dots = document.getElementsByClassName("steps-shower")[0];
for (i=1; i<=slides; i++){
    var span = document.createElement("span");
    span.className = "w3-badge demo w3-border w3-transparent"
    span.onclick = `currentDiv(${i})`
    dots.appendChild(span)
}

/* LOAD GRAPHS */
display_graph = document.getElementById('display_graph')
for (i=0; i<slides; i++) {
    var instance = instances[i]
    /* CREATE EACH GRAPH DIV */
    var graph_div = document.createElement("div");
    graph_div.id  = `graph_${i}`
    graph_div.className = "slide"
    graph_div.style = "width:100%"
    display_graph.appendChild(graph_div)
    /* LOAD cytoscape */
    var cy = (window.cy = cytoscape({
        container: document.getElementById(`graph_${i}`), // container to render in
        elements: [],
        boxSelectionEnabled: false,

        style: [ 

            {
                selector: 'node', 
                css: {
                    "text-valign" : "center", 
                    "text-halign" : "center", 
                    'border-color': '#adcfe6', 
                    'background-color': '#adcfe6', 
                    "border-opacity" : 0.75, 'background-opacity': 0.5, "border-width" : 2.0,
                    'label': 'data(label)', 
                    'width': '100px', 'height': 'data(height)', 
                    "shape" : "ellipse"
                }
            },

            {
                selector: "node[type = 'state']", 
                css: {
                    'background-color': '#FF6347',
                    'shape': "rectangle",
                }
            },

            {   
                selector : "edge",
                css : {
                    "font-size" :'12px',
                    "width" : 2,
                    "line-style" : "solid",
                    "curve-style": "bezier",
                    "source-arrow-shape" : "triangle",
                    "target-arrow-shape" : "triangle",
                    "label": function (edge) { { return `${edge.data('relation')}[${edge.data('pos')}]:: ${edge.data('label')}` } }
                }
            },

            {
                selector: "edge[trace = 'T1']", 
                css: {
                    "line-color" : "#6666ff",
                    'source-arrow-color': '#6666ff',
                    'target-arrow-color': '#6666ff'
                }
            },

            {
                selector: "edge[trace = 'T2']", 
                css: {
                    "line-color" : "#ff8c1a",
                    'source-arrow-color': '#ff8c1a',
                    'target-arrow-color': '#ff8c1a'
                }
            },
        ],
    }));
    /* FILL CY with DATA */
    var nodes = instance['nodes']
    var edges = instance['edges']
    nodes.forEach(node => {
        cy.add({ group: 'nodes', data: node});
    })
    edges.forEach(data => {
        cy.add({ group: 'edges', selectable: true, data: data})
    })
    /* CONFIGURATIONS */
    cy.userZoomingEnabled( false );
    cy.nodes().forEach(function( n ){ n.data('height', n.width()); });
    cy.layout({ name: 'cose-bilkent' }).run();
}

var slideIndex = 1;
showDivs(slideIndex);
function plusDivs(n) {
  showDivs(slideIndex += n);
}
function currentDiv(n) {
  showDivs(slideIndex = n);
}
function showDivs(n) {
  var i;
  var x = document.getElementsByClassName("slide");
  var dots = document.getElementsByClassName("demo");
  if (n > x.length) {slideIndex = 1}
  if (n < 1) {slideIndex = x.length}
  for (i = 0; i < x.length; i++) {
    x[i].style.display = "none";  
  }
  for (i = 0; i < dots.length; i++) {
    dots[i].className = dots[i].className.replace(" w3-white", "");
  }
  x[slideIndex-1].style.display = "block";  
  dots[slideIndex-1].className += " w3-white";
}