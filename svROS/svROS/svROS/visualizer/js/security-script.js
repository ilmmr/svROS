var nodes = {{nodes}}
var edges = {{edges}}

var cy = (window.cy = cytoscape({
    container: document.getElementById('display_graph'), // container to render in
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
                'label': 'data(name)', 
                'width': '100px', 'height': 'data(height)', 
                "shape" : "ellipse"
            }
        },

        {
            selector: "node[type = 'object']", 
            css: {
                'background-color': 'white',
                'shape': "rectangle",
            }
        },

        {
            selector: "node[rule = 'Allow']", 
            css: {
                'background-color': 'white',
                'shape': "rectangle",
                'border-color': '#FF6347'
            }
        },

        {
            selector: "node[rule = 'Deny']", 
            css: {
                'background-color': 'white',
                'shape': "rectangle",
                'border-color': '#262626'
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
                "label": 'data(role)'
            }
        },

        {
            selector: "edge[call = 'privilege']", 
            css: {
                "line-color" : "#666",
                'source-arrow-color': '#666',
                'target-arrow-color': '#666'
            }
        },

        {
            selector: "edge[call = 'source call']", 
            css: {
                "line-color" : "orange",
                'source-arrow-color': 'orange',
                'target-arrow-color': 'orange'
            }
        },

        {
            selector: "edge[call = 'no privilege']", 
            css: {
                "line-style" : "dotted",
                "line-color" : "#666",
                'source-arrow-color': '#666',
                'target-arrow-color': '#666'
            }
        },
    ],
}));
/* FILL CY with DATA */
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