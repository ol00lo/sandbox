Unicode true
Name "PictApp"
OutFile "PictApp_Installer.exe"
InstallDir "$PROGRAMFILES\PictApp"
InstallDirRegKey HKCU "Software\PictApp" "Install_Dir"

!include MUI2.nsh

!define MUI_ABORTWARNING
!define MUI_ICON "resources\p.ico"
!define MUI_UNICON "resources\p.ico"

!define MUI_WELCOMEPAGE_TITLE "Welcome to PictApp Installation"
!define MUI_WELCOMEPAGE_TEXT "A programmer's wife asks:$\r$\n$\r$\n\
'Would you go to the store and buy a loaf of bread?$\r$\n\
And if they have eggs, get ten.'$\r$\n$\r$\n\
He returns with 10 loaves of bread.$\r$\n$\r$\n\
'Why??' she exclaims.$\r$\n$\r$\n\
'They had eggs.'$\r$\n$\r$\n$\r$\n\
Click Next to begin installing PictApp."
!define MUI_WELCOMEFINISHPAGE_BITMAP "resources\welcomepage.bmp"

!insertmacro MUI_PAGE_WELCOME
!insertmacro MUI_PAGE_DIRECTORY

!define MUI_PAGE_CUSTOMFUNCTION_SHOW ComponentsShow
!insertmacro MUI_PAGE_COMPONENTS
!insertmacro MUI_PAGE_INSTFILES

!define MUI_FINISHPAGE_RUN "$INSTDIR\PictApp.exe"
!insertmacro MUI_PAGE_FINISH

!insertmacro MUI_LANGUAGE "English"

Section "Program" SecMain
    SectionIn RO
    
    SetOutPath "$INSTDIR"
    File /r "dist\PictApp\*.*"
    
    WriteRegStr HKCU "Software\PictApp" "Install_Dir" "$INSTDIR"
    
    WriteUninstaller "$INSTDIR\Uninstall.exe"
    
    WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\PictApp" \
        "DisplayName" "PictApp"
    WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\PictApp" \
        "UninstallString" '"$INSTDIR\Uninstall.exe"'
    WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\PictApp" \
        "DisplayIcon" '"$INSTDIR\PictApp.exe"'
SectionEnd

Section "Desktop shortcut." SecDesktop
    CreateShortcut "$DESKTOP\PictApp.lnk" "$INSTDIR\PictApp.exe"
SectionEnd

Section "Start Menu shortcut." SecStartMenu
    CreateDirectory "$SMPROGRAMS\PictApp"
    CreateShortcut "$SMPROGRAMS\PictApp\PictApp.lnk" "$INSTDIR\PictApp.exe"
    CreateShortcut "$SMPROGRAMS\PictApp\Delete PictApp.lnk" "$INSTDIR\Uninstall.exe"
SectionEnd

Section "Uninstall"
    RMDir /r "$INSTDIR"

    Delete "$DESKTOP\PictApp.lnk"
    RMDir /r "$SMPROGRAMS\PictApp"

    DeleteRegKey HKCU "Software\PictApp"
    DeleteRegKey HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\PictApp"
SectionEnd

Function ComponentsShow
	!insertmacro MUI_HEADER_TEXT "Additional Options" "Choose where to create shortcuts"
    GetDlgItem $0 $HWNDPARENT 1
    SendMessage $0 ${WM_SETTEXT} 0 "STR:Install"
FunctionEnd

LangString DESC_SecMain ${LANG_ENGLISH} "Main program components."
LangString DESC_SecDesktop ${LANG_ENGLISH} "Create desktop shortcut."
LangString DESC_SecStartMenu ${LANG_ENGLISH} "Create Start Menu shortcut."

!insertmacro MUI_FUNCTION_DESCRIPTION_BEGIN
    !insertmacro MUI_DESCRIPTION_TEXT ${SecMain} $(DESC_SecMain)
    !insertmacro MUI_DESCRIPTION_TEXT ${SecDesktop} $(DESC_SecDesktop)
    !insertmacro MUI_DESCRIPTION_TEXT ${SecStartMenu} $(DESC_SecStartMenu)
!insertmacro MUI_FUNCTION_DESCRIPTION_END