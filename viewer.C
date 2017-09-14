#include "nuphase.h" 

TGraph * makeGraph(const nuphase_event_t * ev, int ibd = 0, int ch = 0) 
{
  TGraph * g = new TGraph(ev->buffer_length); 

  for (int i = 0; i < g->GetN(); i++)
  {
    g->GetX()[i] = i/1.5; 
    g->GetY()[i] = ev->data[ibd][ch][i]; 
  }
   g->GetXaxis()->SetTitle("t (ns)"); 
   g->GetYaxis()->SetTitle("adu"); 
   g->SetTitle(TString::Format("%s ch %d", ibd == 0 ? "MASTER" : "SLAVE", ch)); 
   g->SetName(TString::Format("g_%d_%d",ibd,ch));  
   g->SetEditable(false); 

   return g; 
}


void viewer(const char * hdfile, const char * evfile, int i = 0) 
{

  FILE * hdf = fopen(hdfile,"r"); 
  FILE * evf = fopen(evfile,"r"); 

  nuphase_event_t ev; 
  nuphase_header_t hd; 

  for (int c = 0; c <= i; c++)
  {
    nuphase_event_read(evf, &ev); 
    nuphase_header_read(hdf, &hd); 
  }

  nuphase_header_print(stdout, &hd); 
  
  for (int ibd = 0; ibd < 2; ibd++)
  {
    if (ev.board_id[ibd])
    {

      TString name = TString::Format("c_%zu_%d", ev.event_number, ev.board_id[ibd]);
      TString title = TString::Format("ev %zu bd%d", ev.event_number, ev.board_id[ibd]);
      TCanvas *c = new TCanvas(name,title, 1800,1000); 
      c->Divide(2,4); 

      for (int ich = 0; ich < NP_NUM_CHAN; ich++)
      {
        c->cd(ich+1); 
        if (hd.channel_read_mask[ibd] & (1 << ich))
        {
          TGraph * g = makeGraph(&ev, ibd, ich); 
          g->Draw("alp"); 
        }
      }
    }
  }

}
