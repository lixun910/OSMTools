//
// Created by Xun Li on 12/23/18.
//
#include <wx/notebook.h>
#include <wx/artprov.h>
#include <wx/filename.h>

#include "Roads.h"
#include "uiRoadDownload.h"

using namespace OSMTools;

uiRoadDownload::uiRoadDownload(const wxString& title)
    : wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, wxSize(500, 300))
{
    InitControls();
}

uiRoadDownload::uiRoadDownload(double top, double bottom,
        double left, double right, const wxString &title)
    : wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, wxSize(500, 300))
{
    InitControls();

    wxString s_top, s_bottom, s_left, s_right;
    s_top << top;
    s_bottom << bottom;
    s_left << left;
    s_right << right;
    tc_bbox_up->SetValue(s_top);
    tc_bbox_bottom->SetValue(s_bottom);
    tc_bbox_left->SetValue(s_left);
    tc_bbox_right->SetValue(s_right);
}

void uiRoadDownload::InitControls()
{
    // main body
    wxPanel *panel = new wxPanel(this, -1);

    wxNotebook *nb = new wxNotebook(panel, -1);

    // bbox page:
    wxPanel *page_bbox = new wxPanel(nb, -1);
    wxStaticBox *st_bbox = new wxStaticBox(page_bbox, -1, _("Extent"),
            wxDefaultPosition, wxSize(300,100));
    wxSize latlon_sz(100,-1);
    tc_bbox_up = new wxTextCtrl(st_bbox, wxID_ANY, _("Top"),
            wxDefaultPosition, latlon_sz);
    tc_bbox_left = new wxTextCtrl(st_bbox, wxID_ANY, _("Left"),
            wxDefaultPosition, latlon_sz);
    tc_bbox_right = new wxTextCtrl(st_bbox, wxID_ANY, _("Right"),
            wxDefaultPosition, latlon_sz);
    tc_bbox_bottom = new wxTextCtrl(st_bbox, wxID_ANY, _("Bottom"),
            wxDefaultPosition, latlon_sz);
    wxGridSizer* bbox_sizer = new wxGridSizer(3, 3, 10, 10);
    bbox_sizer->Add(new wxStaticText(st_bbox, -1, ""));
    bbox_sizer->Add(tc_bbox_up, 0, wxEXPAND);
    bbox_sizer->Add(new wxStaticText(st_bbox, -1, ""));
    bbox_sizer->Add(tc_bbox_left, 0, wxEXPAND);
    bbox_sizer->Add(new wxStaticText(st_bbox, -1, ""));
    bbox_sizer->Add(tc_bbox_right, 0, wxEXPAND);
    bbox_sizer->Add(new wxStaticText(st_bbox, -1, ""));
    bbox_sizer->Add(tc_bbox_bottom, 0, wxEXPAND);
    bbox_sizer->Add(new wxStaticText(st_bbox, -1, ""));
    st_bbox->SetSizer(bbox_sizer);
    wxBoxSizer* bbox_v_sizer = new wxBoxSizer(wxVERTICAL);
    bbox_v_sizer->Add(st_bbox, 0, wxEXPAND, 10);
    page_bbox->SetSizer(bbox_v_sizer);
    nb->AddPage(page_bbox, _("Bounding Box"));

    // input file page:
    wxPanel *infile_panel = new wxPanel(nb, -1);
    wxBoxSizer* infile_h_sizer = new wxBoxSizer(wxHORIZONTAL);
    infile_h_sizer->Add(new wxStaticText(infile_panel, -1, _("Select a file:")));
    tc_infile_path = new wxTextCtrl(infile_panel, -1, "",
            wxDefaultPosition, wxSize(300, -1));
    infile_h_sizer->Add(tc_infile_path);
    wxBitmap bmp = wxArtProvider::GetBitmap(wxART_FILE_OPEN);
    btn_open_file = new wxBitmapButton(infile_panel, -1,
            bmp, wxDefaultPosition, wxSize(18,18));
    infile_h_sizer->Add(btn_open_file, 0, wxLEFT, 10);
    wxStaticBox *st = new wxStaticBox(infile_panel, -1, _("Options:"));
    rb_bbox = new wxRadioButton(st, -1,
            _("Use bounding box of input datasource"),
            wxDefaultPosition, wxDefaultSize, wxRB_GROUP);
    rb_outline = new wxRadioButton(st, -1,
            _("Use outline of input datasource"));
    rb_bbox_outline = new wxRadioButton(st, -1,
            _("Use outline + motorway(~bbox) of input datasource"));
    wxBoxSizer* stb_v_sizer = new wxBoxSizer(wxVERTICAL);
    stb_v_sizer->Add(rb_bbox, 0, wxALL, 5);
    stb_v_sizer->Add(rb_outline, 0, wxALL, 5);
    stb_v_sizer->Add(rb_bbox_outline, 0, wxALL, 5);
    st->SetSizer(stb_v_sizer);
    wxBoxSizer* infile_v_sizer = new wxBoxSizer(wxVERTICAL);
    infile_v_sizer->Add(infile_h_sizer,0, wxTOP, 10);
    infile_v_sizer->Add(st, 1, wxTOP | wxEXPAND, 10);

    infile_panel->SetSizer(infile_v_sizer);
    nb->AddPage(infile_panel, _("Input data source"));

    wxBoxSizer *hbox1 = new wxBoxSizer(wxHORIZONTAL);
    hbox1->Add(nb, 1, wxEXPAND);

    wxBoxSizer *hbox2 = new wxBoxSizer(wxHORIZONTAL);
    wxButton *ok = new wxButton(panel, -1, _("Ok"));
    wxButton *cancel = new wxButton(panel, -1, _("Cancel"));
    hbox2->Add(ok);
    hbox2->Add(cancel);

    wxBoxSizer *hbox3 = new wxBoxSizer(wxHORIZONTAL);
    cb_buffer = new wxCheckBox(panel, -1, _("Buffer query area:"));
    tc_buffer = new wxTextCtrl(panel, -1, "20");
    cb_buffer->SetValue(true);
    hbox3->Add(cb_buffer);
    hbox3->Add(tc_buffer, 0, wxRIGHT, 5);
    hbox3->Add(new wxStaticText(panel, -1, "%"));

    wxBoxSizer *hbox4 = new wxBoxSizer(wxHORIZONTAL);
    ch_way_type = new wxChoice(panel, -1);
    ch_way_type->Append("Drive");
    ch_way_type->SetSelection(0);
    hbox4->Add(new wxStaticText(panel, -1, "Select road type:"));
    hbox4->Add(ch_way_type, 0, wxLEFT, 5);

    wxBoxSizer *vbox = new wxBoxSizer(wxVERTICAL);
    vbox->Add(hbox1, 1, wxEXPAND);
    vbox->Add(hbox3, 0, wxEXPAND | wxALL, 10);
    vbox->Add(hbox4, 0, wxEXPAND | wxALL, 10);
    vbox->Add(hbox2, 0, wxALIGN_RIGHT | wxRIGHT | wxBOTTOM, 10);
    panel->SetSizer(vbox);

    Centre();

    ok->Bind(wxEVT_BUTTON, &uiRoadDownload::OnOKClick, this);
    cancel->Bind(wxEVT_BUTTON, &uiRoadDownload::OnCancelClick, this);
}

bool uiRoadDownload::CheckInput(wxTextCtrl *tc, double& val)
{
    wxString tmp_val, error_msg;

    tmp_val = tc->GetValue();
    if (tmp_val.IsEmpty() ) {
        error_msg = _("Please enter values of bounding box.");
    } else {
        if (tmp_val.ToDouble(&val) == false) {
            error_msg = _("Please check input values are valid");
        }
    }
    if (error_msg.IsEmpty() == false) {
        wxMessageDialog msg_dlg(this, error_msg,
                _("Error"),
                wxOK | wxOK_DEFAULT | wxICON_INFORMATION);
        msg_dlg.ShowModal();
        return false;
    }
    return true;
}

void uiRoadDownload::OnOKClick(wxCommandEvent& event)
{
    double lat_min, lng_min, lat_max, lng_max;
    if (CheckInput(tc_bbox_right, lat_max) == false) return;
    if (CheckInput(tc_bbox_left, lat_min) == false) return;
    if (CheckInput(tc_bbox_up, lng_max) == false) return;
    if (CheckInput(tc_bbox_bottom, lng_min) == false) return;

    if (cb_buffer->IsChecked()) {
        long buffer_val;
        wxString buffer_txt = tc_buffer->GetValue();
        if (buffer_txt.ToLong(&buffer_val) ) {
            double buffer_ratio = 1;
            buffer_ratio = 1 + ((double)buffer_val / 100.0);
            double w = lat_max - lat_min;
            double h = lng_max - lng_min;
            double offset_w = (w * buffer_ratio - w) / 2.0;
            double offset_h = (h * buffer_ratio - h) / 2.0;
            lat_min = lat_min - offset_w;
            lat_max = lat_max + offset_w;
            lng_min = lng_min - offset_h;
            lng_max = lng_max + offset_h;
        }
    }
    wxString save_ttl = _("Save OSM roads file");
    wxString filter = "ESRI Shapefiles (*.shp)|*.shp|JSON files (*.json)|*.json";

    wxFileDialog save_dlg(this, save_ttl, "", "", filter,
            wxFD_SAVE|wxFD_OVERWRITE_PROMPT);

    if (save_dlg.ShowModal() != wxID_OK) return;

    wxFileName fname = wxFileName(save_dlg.GetPath());
    wxString out_fname = fname.GetPathWithSep() + fname.GetName();
    wxString out_type = fname.GetExt();
    wxString json_fpath = out_fname + ".json";
    wxString shp_fpath = out_fname + ".shp";

    OSMTools::Roads roads;
    roads.DownloadByBoundingBox(lat_min, lng_min, lat_max, lng_max,
            OSMTools::drive,
            json_fpath.mb_str(wxConvUTF8));
    if (out_type.CmpNoCase("shp")==0) {
        roads.ReadOSMNodes(json_fpath);
        roads.SaveToShapefile(shp_fpath);
    }

    wxMessageDialog msg_dlg(this, _("Save OSM roads to file successfully."),
                            _("Info"),
                            wxOK | wxOK_DEFAULT | wxICON_INFORMATION);
    msg_dlg.ShowModal();
}

void uiRoadDownload::OnCancelClick(wxCommandEvent& event)
{
    Destroy();
}
