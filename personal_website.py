# Written by David Kooi

import os
import json
import base64

from flask import Flask, request, render_template, url_for
from flask_restful import Resource, Api


from pages.Home     import Homepage
from pages.CV       import CV
from pages.Writing  import Writing
from pages.Projects import ProjectPage 
from pages.Photos   import Photos

from utils.page_manager import PageManager

app = Flask(__name__)
api = Api(app)

# Setup Config
app.config.from_object(__name__)
app.config['root_path']   = app.root_path
app.config['static_path'] = os.path.join(app.root_path, 'static')
app.config['text_path']   = os.path.join(app.config['static_path'],'text')

# Setup Pages
page_manager = PageManager.GetInstance()
page_list = [Homepage, CV, Writing, Photos, ProjectPage]

# Register pages to app and page_manager
for page in page_list:
    app.register_blueprint(page)
    page_manager.RegisterPage(page)
    page.SetConfig(app)



# Setup REST Resources
class DataReceive(Resource):
    """
    Endpoint for hologram io to send telemetry
    """
    def post(self):
        print("Got post") 
        payload = request.form['payload'] 

        # Payload is a json string
        payload_dict = json.loads(payload) 

        # Data is base64 encoded
        data = base64.b64decode(payload_dict['data'])

        print("Received: {}".format(data))

api.add_resource(DataReceive, '/data_input')



# Provide app with global vars
@app.context_processor
def LayoutLinkProcessor():
    
    # Set main links
    links = []
    home  = {}

    for page in page_list:
 
        links.append({"name":page.GetName(), 
                      "href":page.GetRootUrl()})


    # include_header is true by default
    return dict(links=links, include_header=True)

if __name__ == "__main__":
   app.run(host="0.0.0.0", port=5000)


