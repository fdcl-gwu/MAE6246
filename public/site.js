(function () {
  const config = window.MAE6246_CONFIG || {};
  const owner = (config.githubOwner || "").trim();
  const repository = (config.githubRepository || "").trim();
  const branch = (config.githubBranch || "main").trim();

  function markPending(link) {
    link.removeAttribute("href");
    link.classList.add("is-pending");
    link.setAttribute("aria-disabled", "true");
    link.setAttribute("tabindex", "-1");
    link.querySelector("span").textContent = link.dataset.pendingLabel || "Coming soon";
  }

  document.querySelectorAll(".colab-link[data-notebook]").forEach((link) => {
    const released = link.dataset.released === "true";
    const notebook = link.dataset.notebook;

    if (owner && repository && released && notebook) {
      link.href = `https://colab.research.google.com/github/${owner}/${repository}/blob/${branch}/${notebook}`;
      link.target = "_blank";
      link.rel = "noopener noreferrer";
      link.classList.remove("is-pending");
      link.removeAttribute("aria-disabled");
      link.removeAttribute("tabindex");
      link.querySelector("span").textContent = link.dataset.readyLabel || "Open in Colab";
      return;
    }

    markPending(link);
  });

  document.querySelectorAll(".file-link[data-file]").forEach((link) => {
    if (link.dataset.released === "true") {
      link.href = link.dataset.file;
      link.target = "_blank";
      link.rel = "noopener noreferrer";
      link.classList.remove("is-pending");
      link.removeAttribute("aria-disabled");
      link.removeAttribute("tabindex");
      link.querySelector("span").textContent = link.dataset.readyLabel || "Open file";
      return;
    }

    markPending(link);
  });

  document.querySelectorAll(".notebook-download[data-download-url]").forEach((link) => {
    link.addEventListener("click", async (event) => {
      event.preventDefault();

      const label = link.querySelector("span");
      const originalLabel = label.textContent;
      label.textContent = "Preparing download…";
      link.setAttribute("aria-busy", "true");

      try {
        const response = await fetch(link.dataset.downloadUrl);
        if (!response.ok) {
          throw new Error(`Download failed with status ${response.status}`);
        }

        const notebook = await response.blob();
        const objectUrl = URL.createObjectURL(notebook);
        const saveLink = document.createElement("a");
        saveLink.href = objectUrl;
        saveLink.download = link.dataset.filename || "notebook.ipynb";
        document.body.appendChild(saveLink);
        saveLink.click();
        saveLink.remove();
        window.setTimeout(() => URL.revokeObjectURL(objectUrl), 1000);
      } catch (error) {
        console.error(error);
        window.alert("The notebook could not be downloaded. Please try again.");
      } finally {
        label.textContent = originalLabel;
        link.removeAttribute("aria-busy");
      }
    });
  });
})();
