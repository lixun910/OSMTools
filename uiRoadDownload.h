//
// Created by Xun Li on 12/23/18.
//

#ifndef OSMTOOLSPROJECT_UIROADDOWNLOAD_H
#define OSMTOOLSPROJECT_UIROADDOWNLOAD_H

#include <wx/wx.h>

namespace OSMTools {

    class uiRoadDownload : public wxFrame {

    public:
        uiRoadDownload(const wxString &title = _("Download OSM Roads"));

        uiRoadDownload(double top, double bottom,
                double left, double right,
                const wxString &title = _("Download OSM Roads"));

    protected:
        void InitControls();

        bool CheckInput(wxTextCtrl *tc, double& val);

        void OnOKClick(wxCommandEvent& event);

        void OnCancelClick(wxCommandEvent& event);

    protected:
        wxTextCtrl *tc_bbox_up;
        wxTextCtrl *tc_bbox_left;
        wxTextCtrl *tc_bbox_right;
        wxTextCtrl *tc_bbox_bottom;
        wxTextCtrl *tc_infile_path;
        wxBitmapButton *btn_open_file;
        wxRadioButton *rb_bbox;
        wxRadioButton *rb_outline;
        wxRadioButton *rb_bbox_outline;
    };
}

#endif //OSMTOOLSPROJECT_UIROADDOWNLOAD_H
