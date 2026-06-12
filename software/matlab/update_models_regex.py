import zipfile
import os
import shutil
import re

def modify_slx(slx_path, is_lqr=False):
    print(f"Modifying: {slx_path}")
    
    # Restore from backup if it exists, to work on a clean file
    bak_path = slx_path + ".bak"
    if os.path.exists(bak_path):
        print(f"  Restoring clean backup from {bak_path}")
        shutil.copy(bak_path, slx_path)
    
    if not os.path.exists(slx_path):
        print(f"  File not found: {slx_path}")
        return
    
    # Create temp dir
    temp_dir = slx_path + "_extracted"
    if os.path.exists(temp_dir):
        shutil.rmtree(temp_dir)
    os.makedirs(temp_dir)
    
    # Unzip
    with zipfile.ZipFile(slx_path, 'r') as zf:
        zf.extractall(temp_dir)
        
    # 1. Modify system_5.xml (Dynamics: Int_Omega and Int_Omega_w -> StateSpace, remove Transport Delay)
    sys5_path = os.path.join(temp_dir, "simulink", "systems", "system_5.xml")
    if os.path.exists(sys5_path):
        with open(sys5_path, 'r', encoding='utf-8') as f:
            content = f.read()
            
        # Regex to remove Transport Delay block (SID 43) if present
        pat_delay = r'<Block\s+BlockType="TransportDelay"[^>]*?SID="43">.*?</Block>'
        new_content, count_delay = re.subn(pat_delay, "", content, flags=re.DOTALL)
        
        # Regex to remove line 12 -> 43
        pat_line1 = r'<Line>\s*(<P\s+Name="ZOrder">\d+</P>\s*)?<P\s+Name="Src">12#out:1</P>\s*<P\s+Name="Dst">43#in:1</P>\s*</Line>'
        new_content, count_line1 = re.subn(pat_line1, "", new_content, flags=re.DOTALL)
        
        # Regex to replace line 43 -> 16 & 17 with 12 -> 16 & 17
        pat_line2 = r'<Line>\s*<P\s+Name="ZOrder">\d+</P>\s*<P\s+Name="Src">43#out:1</P>.*?</Line>'
        repl_line2 = """<Line>
    <P Name="ZOrder">3</P>
    <P Name="Src">12#out:1</P>
    <Branch>
      <P Name="ZOrder">2</P>
      <P Name="Dst">16#in:1</P>
    </Branch>
    <Branch>
      <P Name="ZOrder">4</P>
      <P Name="Dst">17#in:1</P>
    </Branch>
  </Line>"""
        new_content, count_line2 = re.subn(pat_line2, repl_line2, new_content, flags=re.DOTALL)
        
        if count_delay > 0:
            print("  [+] Removed Transport Delay block in system_5.xml")
        if count_line1 > 0:
            print("  [+] Removed old line 12->43 in system_5.xml")
        if count_line2 > 0:
            print("  [+] Rewired lines to bypass Transport Delay in system_5.xml")
            
        # Regex replacement for Int_Omega
        pat_omega = r'<Block\s+BlockType="Integrator"\s+Name="Int_Omega"\s+SID="19">.*?</Block>'
        repl_omega = """<Block BlockType="StateSpace" Name="Int_Omega" SID="19">
    <PortCounts in="1" out="1" />
    <P Name="Position">[280, 90, 310, 120]</P>
    <P Name="ZOrder">8</P>
    <P Name="FontName">Andale Mono</P>
    <P Name="A">-tau</P>
    <P Name="B">1</P>
    <P Name="C">1</P>
    <P Name="D">0</P>
    <P Name="X0">omega0</P>
  </Block>"""
        
        # Regex replacement for Int_Omega_w
        pat_omega_w = r'<Block\s+BlockType="Integrator"\s+Name="Int_Omega_w"\s+SID="18">.*?</Block>'
        repl_omega_w = """<Block BlockType="StateSpace" Name="Int_Omega_w" SID="18">
    <PortCounts in="1" out="1" />
    <P Name="Position">[280, 160, 310, 190]</P>
    <P Name="ZOrder">7</P>
    <P Name="FontName">Andale Mono</P>
    <P Name="A">-tau</P>
    <P Name="B">1</P>
    <P Name="C">1</P>
    <P Name="D">0</P>
    <P Name="X0">omega_w0</P>
  </Block>"""
        
        new_content, count1 = re.subn(pat_omega, repl_omega, new_content, flags=re.DOTALL)
        new_content, count2 = re.subn(pat_omega_w, repl_omega_w, new_content, flags=re.DOTALL)
        
        if count1 > 0:
            print("  [+] Replaced Int_Omega in system_5.xml")
        if count2 > 0:
            print("  [+] Replaced Int_Omega_w in system_5.xml")
            
        with open(sys5_path, 'w', encoding='utf-8') as f:
            f.write(new_content)
            
    # 2. Modify system_2.xml (Controller: add Hysteresis_Relay InitialState to 1 for both, add PID for PD only)
    sys2_path = os.path.join(temp_dir, "simulink", "systems", "system_2.xml")
    if os.path.exists(sys2_path):
        with open(sys2_path, 'r', encoding='utf-8') as f:
            content = f.read()
            
        # Set InitialState of Hysteresis_Relay to 1
        pat_relay = r'(<Block BlockType="Relay" Name="Hysteresis_Relay"[^>]*?>.*?<P Name="OffSwitchValue">omega_th_low</P>)'
        repl_relay = r'\1\n    <P Name="InitialState">1</P>'
        new_content, count_relay = re.subn(pat_relay, repl_relay, content, flags=re.DOTALL)
        if count_relay > 0:
            print("  [+] Set Hysteresis_Relay InitialState to 1 in system_2.xml")
            
        if not is_lqr:
            # Regex replacement for Sum_PD
            pat_sum_pd = r'<Block\s+BlockType="Sum"\s+Name="Sum_PD"\s+SID="28">.*?</Block>'
            repl_sum_pid = """<Block BlockType="Sum" Name="Sum_PID" SID="28">
    <PortCounts in="3" out="1" />
    <P Name="Position">[350, 95, 370, 115]</P>
    <P Name="ZOrder">8</P>
    <P Name="FontName">Andale Mono</P>
    <P Name="Inputs">++-</P>
  </Block>
  <Block BlockType="Integrator" Name="Int_Error" SID="46">
    <PortCounts in="1" out="1" />
    <P Name="Position">[220, 180, 250, 210]</P>
    <P Name="ZOrder">20</P>
    <P Name="FontName">Andale Mono</P>
    <P Name="InitialCondition">0</P>
  </Block>
  <Block BlockType="Gain" Name="Ki_Gain" SID="47">
    <P Name="Position">[280, 180, 320, 210]</P>
    <P Name="ZOrder">21</P>
    <P Name="FontName">Andale Mono</P>
    <P Name="Gain">Ki</P>
  </Block>"""
            
            # Line from Wrap_To_Pi to Kp_Gain
            pat_line_wrap = r'<Line>\s*<P\s+Name="ZOrder">\d+</P>\s*<P\s+Name="Src">42#out:1</P>\s*<P\s+Name="Dst">26#in:1</P>\s*</Line>'
            repl_line_wrap = """<Line>
      <P Name="ZOrder">48</P>
      <P Name="Src">42#out:1</P>
      <Branch>
        <P Name="Dst">26#in:1</P>
      </Branch>
      <Branch>
        <P Name="Dst">46#in:1</P>
      </Branch>
    </Line>"""
            
            # Line from Kd_Gain to Sum_PD
            pat_line_kd = r'<Line>\s*<P\s+Name="ZOrder">\d+</P>\s*<P\s+Name="Src">27#out:1</P>\s*<P\s+Name="Points">\[.*?\]</P>\s*<P\s+Name="Dst">28#in:2</P>\s*</Line>'
            repl_line_kd = """<Line>
      <P Name="ZOrder">10</P>
      <P Name="Src">27#out:1</P>
      <P Name="Points">[95, 0]</P>
      <P Name="Dst">28#in:3</P>
    </Line>
    <Line>
      <P Name="ZOrder">50</P>
      <P Name="Src">46#out:1</P>
      <P Name="Dst">47#in:1</P>
    </Line>
    <Line>
      <P Name="ZOrder">51</P>
      <P Name="Src">47#out:1</P>
      <P Name="Dst">28#in:2</P>
    </Line>"""
            
            new_content, count_sum = re.subn(pat_sum_pd, repl_sum_pid, new_content, flags=re.DOTALL)
            new_content, count_wrap = re.subn(pat_line_wrap, repl_line_wrap, new_content, flags=re.DOTALL)
            new_content, count_kd = re.subn(pat_line_kd, repl_line_kd, new_content, flags=re.DOTALL)
            
            if count_sum > 0:
                print("  [+] Replaced Sum_PD with Sum_PID and added Int_Error/Ki_Gain in system_2.xml")
            if count_wrap > 0:
                print("  [+] Branched line from Wrap_To_Pi to also go to Int_Error")
            if count_kd > 0:
                print("  [+] Updated Kd_Gain line and added new PID lines")
                
        with open(sys2_path, 'w', encoding='utf-8') as f:
            f.write(new_content)
                
    # Re-zip
    if not os.path.exists(bak_path):
        os.rename(slx_path, bak_path)
    else:
        os.remove(slx_path)
        
    with zipfile.ZipFile(slx_path, 'w', zipfile.ZIP_DEFLATED) as zf:
        for root, dirs, files in os.walk(temp_dir):
            for file in files:
                file_path = os.path.join(root, file)
                arcname = os.path.relpath(file_path, temp_dir)
                zf.write(file_path, arcname)
                
    # Clean up extraction directory
    shutil.rmtree(temp_dir)
    print(f"  Finished modifying: {slx_path}\n")

if __name__ == "__main__":
    targets = [
        ("/Users/bbtp/Desktop/THESIS/CUBESAT_THESI_VS/software/simulink/Cubesat_Control_PD.slx", False),
        ("/Users/bbtp/Desktop/THESIS/CUBESAT_THESI_VS/software/matlab/Cubesat_Control_PD.slx", False),
        ("/Users/bbtp/Desktop/THESIS/CUBESAT_THESI_VS/software/simulink/Cubesat_Control_LQR.slx", True),
        ("/Users/bbtp/Desktop/THESIS/CUBESAT_THESI_VS/software/matlab/Cubesat_Control_LQR.slx", True),
    ]
    for path, is_lqr in targets:
        modify_slx(path, is_lqr)
