from pyqtlet2 import L, leaflet


def init_map_cursor(lmap: L.map):
  js = """
    var styleSheet = document.createElement("style")
    styleSheet.type = "text/css"
    styleSheet.innerText = ".leaflet-container { cursor:crosshair; }"
    document.head.appendChild(styleSheet)
  """
  lmap.runJavaScript(js, 0)


def init_map_scale(lmap: L.map):
  js = '''
    var scaleControl = L.control.scale({
      position: 'bottomright',
      metric: true,
      imperial: false,
      updateWhenIdle: false,
      updateWhenZooming: true,
      updateWhenDragging: false
    });
    scaleControl.addTo(map);
  '''
  lmap.runJavaScript(js, 0)

def init_map_set_robot_icon(lmap: L.map, rover_icon_path: str, jsName: str):
  lmap.runJavaScript(
    f'''
    try {{
      var roverIcon = L.icon({{
        iconUrl: "{rover_icon_path}",
        iconSize: [26, 26],
        iconAnchor: [13, 13]
      }});
      
      // Make sure we have a valid reference before setting the icon
      if ({jsName} != undefined) {{
        {jsName}.setIcon(roverIcon);
        console.log("Robot icon set successfully");
      }} else {{
        console.error("Robot marker reference not found: {jsName}");
      }}
    }} catch(e) {{
      console.error("Error setting robot icon:", e);
    }}
  ''', 0)

def map_marker_set_icon(lmap: L.map, marker_icon_path: str, jsName: str):
  lmap.runJavaScript(
    f'''{jsName}.setIcon(L.icon({{"iconUrl": "{marker_icon_path}"}}));''', 0
  )

def map_marker_set_lat_lng_points(lmap: L.map, lat_lng_points: list[list[float]], jsName: str):
	lmap.runJavaScript(f'{jsName}.setLatLngs({lat_lng_points});', 0)

def map_marker_set_lat_lng_point(lmap: L.map, lat_lng_points: list[float], jsName: str):
	lmap.runJavaScript(f'{jsName}.setLatLng({lat_lng_points});', 0)

def map_marker_set_radius(lmap: L.map, radius: float, jsName: str):
  lmap.runJavaScript(f'{jsName}.setRadius({radius});', 0)

def map_line_redraw(lmap: L.map, jsName: str):
  lmap.runJavaScript(f'{jsName}.redraw();', 0)

def map_layer_bind_tooltip(layer: leaflet.layer.Layer, content: str, options=None):
	js = '{layerName}.bindTooltip("{content}"'.format(
			layerName=layer._layerName, content=content)
	if options:
		js += ', {options}'.format(options=layer._stringifyForJs(options))
	js += ')'
	layer.runJavaScript(js, 0)
	return layer